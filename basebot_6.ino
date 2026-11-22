/***************************************************************************
*   Copyright (C) 2024 by DTU                                              *
*   jcan@dtu.dk                                                            *
*                                                                          *
*   Base Teensy firmware - Final Project Keyhole Controller                *
*   build for Teensy 4.1, intended for digital control course (34765)      *
***************************************************************************/
#include <Arduino.h>
void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

// Sampling and PWM configuration
const uint32_t sampleTimeUs = 1000;      // Ts = 1 ms (1 kHz)
const uint32_t experimentPWMfrq = 77824; // ~78 kHz ripple-free PWM

// Robot geometry
const float gear = 9.6f;
const float wheelRadius = 0.0315f;       // (m)
const float wheelBase = 0.15f;           // (m)

// Odometry & pose states
float distA = 0.0f;                      // Traveled distance s (m)
float pose[4];                           // [0]: x (m), [1]: y (m), [2]: h_enc (rad), [3]: h_gyro (rad)
float poseVel[2];                        // [0]: v (m/s), [1]: w_enc (rad/s)
float poseVel_prev[2] = {0.0f, 0.0f};
float vx_prev = 0.0f;
float vy_prev = 0.0f;

// 22-deg tilted IMU gyroscope state
const float imuTiltRad = 22.0f * float(M_PI) / 180.0f;
float gyroZ_offset = 0.0f;
bool gyroOffsetReady = false;
float gyroTurnrate = 0.0f;
float gyroTurnrate_prev = 0.0f;
float gyroHeading = 0.0f;

// Inner Loop: Wheel Velocity PI Controller
const float Kp_vel = 0.02f;              // V / (rad/s)
const float taui_vel = 0.04f;            // (s)
const float maxMotorVolt = 9.0f;         // (V)
float velErr_prev[2] = {0.0f, 0.0f};
float velInt[2] = {0.0f, 0.0f};
float inVelLeft = 0.0f;
float inVelRight = 0.0f;
bool velSaturated = false;

// Outer Loop: Heading Controller & Acceleration Limiters
const float Kp_head = 3.5f;              // (rad/s) / rad
const float Kd_head = 0.55f;             // (s)
const float maxTurnrate = 3.2f;          // (rad/s)
float headU = 0.0f;                      // Wheel velocity offset (m/s)
float turnRateFF = 0.0f;                 // Curvature feedforward w_ff = v/R (rad/s)
float stepStartDist = 0.0f;
int logTickCnt = 0;

const float Kp_acc = 25.0f;              // Acceleration limiter gain (1/s)
const float maxAccVel = 2.0f;            // Max forward acceleration (m/s^2)
const float maxAccTurn = 1.0f;           // Max turn acceleration after B/2 (m/s^2)
float accVel = 0.0f, accVel_prev = 0.0f, velLim = 0.0f;
float accTurn = 0.0f, accTurn_prev = 0.0f, turnLim = 0.0f;

// Final Project "Keyhole" Course Geometry (Lecture 10, Slide 3)
// 1.0 m Straight -> Turn -72.54 deg -> Circle (R = 0.50 m, L = 2.837 m) -> Turn -72.54 deg -> 1.5 m Straight
const float fp_L1        = 1.00f;
const float fp_L2        = 1.50f;
const float fp_R         = 0.50f;
const float fp_xc        = 1.47697f;     // 1.00 + sqrt(0.50^2 - 0.15^2) (m)
const float fp_yc        = 0.15000f;     // (m)
const float fp_hTan1     = -1.26610f;    // -(pi/2 - asin(0.15/0.50)) = -72.54 deg (rad)
const float fp_dTheta    = 5.67380f;     // 2*pi - 2*asin(0.15/0.50)  = 325.08 deg (rad)
const float fp_Larc      = 2.83690f;     // R * fp_dTheta (m)
const float fp_hTan2     = 4.40770f;     // fp_hTan1 + fp_dTheta = +252.54 deg (rad)
const float fp_hExit     = float(M_PI);  // +180.0 deg (rad)
const float fp_vStraight = 0.75f;        // (m/s)
const float fp_vCircle   = 0.65f;        // (m/s)
const float fp_dBrakeStr = (fp_vStraight * fp_vStraight) / (2.0f * maxAccVel) + 0.050f * fp_vStraight;
const float fp_dBrakeCir = (fp_vCircle * fp_vCircle)     / (2.0f * maxAccVel) + 0.050f * fp_vCircle;

// Data logging structure
typedef struct
{
  float time;
  int state;
  float desiredValue;
  float desiredHeading;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float pose[4];
  float distA;
  float poseVel[2];
  float inVelLeft;
  float inVelRight;
  float gyro[3];
  float encDelay[2];
  float headU;
  float gyroTurnrate;
  float accVel;
  float accTurn;
} LogData;

const int logsMax = 400000 / sizeof(LogData);
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0;

void printLog()
{
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, PWM ");  Serial.print(experimentPWMfrq);
  Serial.print(" Hz, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   Desired velocity (m/s)");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11-13 gyro (x,y,z) (deg/s)");
  Serial.println("% 14-15 motor current  (A)");
  Serial.println("% 16  Forward velocity poseVel[0] (m/s)");
  Serial.println("% 17  Turnrate poseVel[1] (rad/s)");
  Serial.println("% 18  Distance distA (m)");
  Serial.println("% 19  Heading encoder pose[2] (rad)");
  Serial.println("% 20-21 Position x, y (pose[0], pose[1]) (m)");
  Serial.println("% 22  Turnrate controller output headU (m/s)");
  Serial.println("% 23  Gyro turnrate Z tilt-compensated (rad/s)");
  Serial.println("% 24  Gyro heading integrated (rad)");
  Serial.println("% 25  Desired heading / turnrate reference");
  Serial.println("% 26-27 velocity estimate delay (left, right) (ms)");
  Serial.println("% 28-29 Acceleration (forward, turn) (m/s^2)");

  for (int i = 0; i < logsCnt; i++)
  {
    Serial.print(d->time,4);
    Serial.print(" "); Serial.print(d->state);
    Serial.print(" "); Serial.print(d->desiredValue,2);
    Serial.print(" "); Serial.print(d->motorVolt[0],2);
    Serial.print(" "); Serial.print(d->motorVolt[1],2);
    Serial.print(" "); Serial.print(d->encoder[0]);
    Serial.print(" "); Serial.print(d->encoder[1]);
    Serial.print(" "); Serial.print(d->motorVel[0],2);
    Serial.print(" "); Serial.print(d->motorVel[1],2);
    Serial.print(" "); Serial.print(d->battery,2);
    Serial.print(" "); Serial.print(d->gyro[0],2);
    Serial.print(" "); Serial.print(d->gyro[1],2);
    Serial.print(" "); Serial.print(d->gyro[2],2);
    Serial.print(" "); Serial.print(d->motorCurrent[0],4);
    Serial.print(" "); Serial.print(d->motorCurrent[1],4);
    Serial.print(" "); Serial.print(d->poseVel[0],4);
    Serial.print(" "); Serial.print(d->poseVel[1],4);
    Serial.print(" "); Serial.print(d->distA,4);
    Serial.print(" "); Serial.print(d->pose[2],4);
    Serial.print(" "); Serial.print(d->pose[0],4);
    Serial.print(" "); Serial.print(d->pose[1],4);
    Serial.print(" "); Serial.print(d->headU,4);
    Serial.print(" "); Serial.print(d->gyroTurnrate,4);
    Serial.print(" "); Serial.print(d->pose[3],4);
    Serial.print(" "); Serial.print(d->desiredHeading,4);
    Serial.print(" "); Serial.print(d->encDelay[0]*1000,2);
    Serial.print(" "); Serial.print(d->encDelay[1]*1000,2);
    Serial.print(" "); Serial.print(d->accVel,4);
    Serial.print(" "); Serial.print(d->accTurn,4);
    Serial.println("");
    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// Timing & state machine variables
bool isSampleTime = false;
float time_sec = 0;
IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}

int state = 0;
float endTime = 0;
float desiredValue = 0;
float desiredHeading = 0;

void reset()
{
  distA = 0.0f;
  for (int i = 0; i < 4; i++) pose[i] = 0.0f;
  poseVel[0] = 0.0f;
  poseVel[1] = 0.0f;
  poseVel_prev[0] = 0.0f;
  poseVel_prev[1] = 0.0f;
  vx_prev = 0.0f;
  vy_prev = 0.0f;

  if (!gyroOffsetReady)
  {
    gyroZ_offset = imu2.gyro[2];
    gyroOffsetReady = true;
  }
  gyroTurnrate = 0.0f;
  gyroTurnrate_prev = 0.0f;
  gyroHeading = 0.0f;

  velErr_prev[0] = 0.0f;
  velErr_prev[1] = 0.0f;
  velInt[0] = 0.0f;
  velInt[1] = 0.0f;
  inVelLeft = 0.0f;
  inVelRight = 0.0f;
  velSaturated = false;

  headU = 0.0f;
  turnRateFF = 0.0f;
  stepStartDist = 0.0f;
  logTickCnt = 0;

  accVel = 0.0f;
  accVel_prev = 0.0f;
  velLim = 0.0f;
  accTurn = 0.0f;
  accTurn_prev = 0.0f;
  turnLim = 0.0f;
}

void start()
{
  logsCnt = 0;
  reset();
  motor.setPWMfrq(experimentPWMfrq);
  time_sec = 0;
  encoder.encoder[0] = 0;
  encoder.encoder[1] = 0;
  display.useDisplay = false;
}

void stop(float after)
{
  state = 2;
  desiredValue = 0;
  endTime = time_sec + after;
}

void finished()
{
  motor.motorVoltage[0] = 0;
  motor.motorVoltage[1] = 0;
  motor.setPWMfrq(77824);
  display.useDisplay = true;
  state = 0;
  printLog();
}

// Step 1 & Step 3: Differential-drive kinematics, 22-deg IMU gyro & Tustin pose integration
void updatePose()
{
  float T = sampleTimeUs * 1e-6f;

  float vLeft  = (-encoder.motorVelocity[0] / gear) * wheelRadius;
  float vRight = ( encoder.motorVelocity[1] / gear) * wheelRadius;

  poseVel[0] = 0.5f * (vRight + vLeft);
  poseVel[1] = (vRight - vLeft) / wheelBase;

  distA   += (T * 0.5f) * (poseVel[0] + poseVel_prev[0]);
  pose[2] += (T * 0.5f) * (poseVel[1] + poseVel_prev[1]);
  poseVel_prev[0] = poseVel[0];
  poseVel_prev[1] = poseVel[1];

  if (state == 0)
  {
    if (!gyroOffsetReady)
    {
      gyroZ_offset = imu2.gyro[2];
      gyroOffsetReady = true;
    }
    else if (fabsf(encoder.motorVelocity[0]) < 0.05f && fabsf(encoder.motorVelocity[1]) < 0.05f &&
             fabsf(imu2.gyro[2] - gyroZ_offset) < 1.2f)
    {
      gyroZ_offset = 0.995f * gyroZ_offset + 0.005f * imu2.gyro[2];
    }
    gyroTurnrate = 0.0f;
    gyroTurnrate_prev = 0.0f;
    gyroHeading = 0.0f;
  }
  else if (state == 10)
  {
    if (fabsf(imu2.gyro[2] - gyroZ_offset) < 0.8f)
    {
      gyroZ_offset = 0.98f * gyroZ_offset + 0.02f * imu2.gyro[2];
    }
    gyroTurnrate = 0.0f;
    gyroTurnrate_prev = 0.0f;
    gyroHeading = 0.0f;
  }
  else
  {
    gyroTurnrate = ((imu2.gyro[2] - gyroZ_offset) * (float(M_PI) / 180.0f)) / cosf(imuTiltRad);
    gyroHeading += (T * 0.5f) * (gyroTurnrate + gyroTurnrate_prev);
    gyroTurnrate_prev = gyroTurnrate;
  }
  pose[3] = gyroHeading;

  float vx = poseVel[0] * cosf(gyroHeading);
  float vy = poseVel[0] * sinf(gyroHeading);

  pose[0] += (T * 0.5f) * (vx + vx_prev);
  pose[1] += (T * 0.5f) * (vy + vy_prev);
  vx_prev = vx;
  vy_prev = vy;
}

// Final Project "Keyhole" Trajectory State Machine
void SequenceTwoSteps()
{
  bool button;
  switch (state)
  {
    case 0: // Wait for start button or USB 'start'
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      {
        start();
        desiredValue = 0.0f;
        desiredHeading = 0.0f;
        endTime = time_sec + 0.100f;
        state = 10;
      }
      break;

    case 10: // Brief standstill before launch
      if (time_sec > endTime)
      {
        desiredValue = fp_vStraight;
        desiredHeading = 0.0f;
        turnRateFF = 0.0f;
        stepStartDist = distA;
        state = 11;
      }
      break;

    case 11: // Segment 1: Entry Straight (1.0 m along y = 0)
      {
        float s_seg = distA - stepStartDist;
        desiredHeading = constrain(-2.0f * pose[1], -0.10f, 0.10f);
        turnRateFF = 0.0f;
        desiredValue = (s_seg < (fp_L1 - fp_dBrakeStr)) ? fp_vStraight : 0.0f;

        if ((s_seg >= (fp_L1 - fp_dBrakeStr) && fabsf(poseVel[0]) < 0.04f) || s_seg >= (fp_L1 + 0.015f))
        {
          desiredValue = 0.0f;
          desiredHeading = fp_hTan1;
          turnRateFF = 0.0f;
          state = 12;
        }
      }
      break;

    case 12: // Corner 1: Point turn at P1 (1.00, 0.00) to circle tangent (-72.54 deg)
      {
        desiredValue = 0.0f;
        desiredHeading = fp_hTan1;
        turnRateFF = 0.0f;
        if (fabsf(desiredHeading - gyroHeading) < 0.025f && fabsf(gyroTurnrate) < 0.25f)
        {
          stepStartDist = distA;
          desiredValue = fp_vCircle;
          state = 13;
        }
      }
      break;

    case 13: // Segment 2: Circular Loop (R = 0.50 m, L_arc = 2.837 m, center = (1.477, 0.150))
      {
        float s_seg = distA - stepStartDist;
        if (s_seg < 0.0f) s_seg = 0.0f;

        float h_nom = fp_hTan1 + (s_seg / fp_R);
        if (h_nom > fp_hTan2) h_nom = fp_hTan2;
        float dx = pose[0] - fp_xc;
        float dy = pose[1] - fp_yc;
        float r_err = sqrtf(dx * dx + dy * dy) - fp_R;
        desiredHeading = h_nom + constrain(2.0f * r_err, -0.15f, 0.15f);

        float vFwd = (poseVel[0] > 0.0f) ? poseVel[0] : 0.0f;
        turnRateFF = vFwd / fp_R;

        desiredValue = (s_seg < (fp_Larc - fp_dBrakeCir)) ? fp_vCircle : 0.0f;

        if ((s_seg >= (fp_Larc - fp_dBrakeCir) && fabsf(poseVel[0]) < 0.04f) || s_seg >= (fp_Larc + 0.015f))
        {
          desiredValue = 0.0f;
          desiredHeading = fp_hExit;
          turnRateFF = 0.0f;
          state = 14;
        }
      }
      break;

    case 14: // Corner 2: Point turn at P2 (1.00, 0.30) to exit straight (+180 deg)
      {
        desiredValue = 0.0f;
        desiredHeading = fp_hExit;
        turnRateFF = 0.0f;
        if (fabsf(desiredHeading - gyroHeading) < 0.025f && fabsf(gyroTurnrate) < 0.25f)
        {
          stepStartDist = distA;
          desiredValue = fp_vStraight;
          state = 15;
        }
      }
      break;

    case 15: // Segment 3: Exit Straight (1.5 m along y = 0.30 m) to Stop at x = -0.500 m
      {
        float distToStop = pose[0] - (-0.500f);
        desiredHeading = fp_hExit + constrain(2.0f * (pose[1] - 0.300f), -0.12f, 0.12f);
        turnRateFF = 0.0f;
        desiredValue = (distToStop > fp_dBrakeStr) ? fp_vStraight : 0.0f;

        if (distToStop <= 0.005f || (distToStop <= fp_dBrakeStr && fabsf(poseVel[0]) < 0.02f))
        {
          stop(0.5f);
        }
      }
      break;

    default:
      if (time_sec > endTime)
        finished();
      break;
  }
}

// Cascade Controller: Outer Heading PD + FF -> Acceleration Limiters -> Wheel Mixer -> Inner Motor PI
void controlUpdate()
{
  float T = sampleTimeUs * 1e-6f;
  float velRef = desiredValue;

  // 1) Outer Heading Loop
  float e_head = desiredHeading - gyroHeading;
  bool isPointTurn = (state == 12 || state == 14);
  float kd_eff = isPointTurn ? 0.22f : Kd_head;
  float turnRef = turnRateFF + Kp_head * e_head - kd_eff * (gyroTurnrate - turnRateFF);
  if (isPointTurn && fabsf(e_head) > 0.04f && fabsf(turnRef) < 0.60f)
  {
    turnRef = (e_head > 0.0f) ? 0.60f : -0.60f;
  }
  turnRef = constrain(turnRef, -maxTurnrate, maxTurnrate);
  headU = turnRef * (wheelBase * 0.5f);

  // 2) Closed-Loop Acceleration Limiters
  accVel = constrain(Kp_acc * (velRef - velLim), -maxAccVel, maxAccVel);
  velLim += 0.5f * T * (accVel + accVel_prev);
  accVel_prev = accVel;

  accTurn = constrain(Kp_acc * (headU - turnLim), -maxAccTurn, maxAccTurn);
  turnLim += 0.5f * T * (accTurn + accTurn_prev);
  accTurn_prev = accTurn;

  // 3) Differential Wheel Mixer
  inVelLeft  = velLim - turnLim;
  inVelRight = velLim + turnLim;

  // 4) Inner Motor Velocity PI Controllers (Tustin with +/- 9 V Anti-Windup)
  float wRef[2]  = { inVelLeft * (gear / wheelRadius), inVelRight * (gear / wheelRadius) };
  float wMeas[2] = { -encoder.motorVelocity[0], encoder.motorVelocity[1] };
  float uVolt[2] = { 0.0f, 0.0f };
  velSaturated = false;

  for (int m = 0; m < 2; m++)
  {
    float e_vel = wRef[m] - wMeas[m];
    float u_p = Kp_vel * e_vel;
    velInt[m] = constrain(velInt[m] + (Kp_vel * T / (2.0f * taui_vel)) * (e_vel + velErr_prev[m]),
                          -maxMotorVolt, maxMotorVolt);
    float u_total = u_p + velInt[m];
    if (u_total > maxMotorVolt)       { u_total =  maxMotorVolt; velSaturated = true; }
    else if (u_total < -maxMotorVolt) { u_total = -maxMotorVolt; velSaturated = true; }

    velErr_prev[m] = e_vel;
    uVolt[m] = u_total;
  }

  motor.motorVoltage[0] = -uVolt[0];
  motor.motorVoltage[1] =  uVolt[1];
}

void updateLog()
{
  if (state > 0 and logsCnt < logsMax)
  {
    logTickCnt++;
    if ((logTickCnt % 4) != 1) return; // Log at 250 Hz (4 ms) so full ~12 s run fits in RAM

    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0];
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].motorCurrent[0] = -(analogRead(A1) - 2048) * 0.0045 + 0.260;
    logs[logsCnt].motorCurrent[1] =  (analogRead(A0) - 2048) * 0.0045 - 0.275;
    logs[logsCnt].poseVel[0] = poseVel[0];
    logs[logsCnt].poseVel[1] = poseVel[1];
    logs[logsCnt].distA = distA;
    logs[logsCnt].pose[0] = pose[0];
    logs[logsCnt].pose[1] = pose[1];
    logs[logsCnt].pose[2] = pose[2];
    logs[logsCnt].pose[3] = pose[3];
    logs[logsCnt].inVelLeft = inVelLeft;
    logs[logsCnt].inVelRight = inVelRight;
    logs[logsCnt].headU = headU;
    logs[logsCnt].gyroTurnrate = gyroTurnrate;
    logs[logsCnt].accVel = accVel;
    logs[logsCnt].accTurn = accTurn;
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
    logs[logsCnt].encDelay[0] = encoder.encoderDelay[0];
    logs[logsCnt].encDelay[1] = encoder.encoderDelay[1];
    logsCnt++;
  }
}

void setup()
{
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  robot.setup();
  motor.setup();
  encoder.setup();
  encoder.pulsPerRev = 48;
  imu2.setup();
  motor.setPWMfrq(0x13000);
  usb.setup();
  display.useDisplay = true;
}

void loop(void)
{
  while (true)
  {
    if (isSampleTime)
    {
      isSampleTime = false;
      imu2.tick();
      encoder.tick();
      updatePose();
      SequenceTwoSteps();
      if (state > 0)
      {
        controlUpdate();
      }
      motor.tick();
      updateLog();
      robot.tick();
      display.tick();

      static bool wasSerialConnected = false;
      static int serialConnectDelayTicks = 0;
      bool isSerialConnected = bool(Serial);
      if (isSerialConnected && !wasSerialConnected)
      {
        serialConnectDelayTicks = 500;
      }
      wasSerialConnected = isSerialConnected;
      if (serialConnectDelayTicks > 0)
      {
        serialConnectDelayTicks--;
        if (serialConnectDelayTicks == 0 && state == 0 && logsCnt > 0)
        {
          printLog();
        }
      }
    }
    usb.tick();
  }
}
