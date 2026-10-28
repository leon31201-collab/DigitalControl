/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
* 
* The MIT License (MIT)  https://mit-license.org/
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */
#include <Arduino.h>
void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

/**
 * =========================================================================
 * SCHRITT 1: Globale Einstellungen, Geometrie & Regler-Speicher (Tustin)
 * ========================================================================= */
// 1 = spikeSequence (transient, R/L), 0 = SequenceTwoSteps (Driving straight / Square test)
#define USE_SPIKE_SEQUENCE 0
// 1 = plain P-controller per motor (lecture 5, the pstep_* sampling-time logs), 0 = PI cascade
#define USE_P_CONTROL 0
const float Kp_step = 0.03f;          // V per motor rad/s, same units as the Simulink model
// Sample time can not go lower than 300us (1000us = 1ms)
const uint32_t sampleTimeUs = 1000;   // Ts = 1 ms = 0.001 s
// Default high PWM frequency (77824 Hz) for quiet, ripple-free operation
const uint32_t experimentPWMfrq = 77824;

// Robot geometry configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.15;     // (m) distance B between left and right wheel, lecture 6 slide 8

// Pose and kinematic state values
float distA = 0;     // trip distance (m)
float pose[4];       // [0]: x (m), [1]: y (m), [2]: heading h (rad), [3]: gyroHeading (rad)
float poseVel[2];    // [0]: forward velocity v (m/s), [1]: turnrate h_dot (rad/s)
bool velSaturated = false;

// Memory variables (x1 = value from last sample k-1) for Tustin integrators in updatePose()
float poseVel_prev[2] = {0.0f, 0.0f}; // [0]: v(k-1), [1]: turnrate(k-1)
float vx_prev = 0.0f;                 // vx(k-1) = v(k-1)*cos(h(k-1))
float vy_prev = 0.0f;                 // vy(k-1) = v(k-1)*sin(h(k-1))

// Week 8 (Step 1): Gyro-based turnrate (rad/s) and integrated gyro heading (rad)
const float imuTiltRad = 22.0f * float(M_PI) / 180.0f; // ~22 deg mounting tilt of IMU board
float gyroZ_offset = 0.0f;            // Standstill bias of imu2.gyro[2] (deg/s)
float gyroTurnrate = 0.0f;            // Tilt-compensated gyro turnrate (rad/s)
float gyroTurnrate_prev = 0.0f;       // gyroTurnrate(k-1) for Tustin integration
float gyroHeading = 0.0f;             // Integrated gyro heading h_gyro (rad)

// Inner Loop: Motor Velocity PI Controller Parameters
// Note: Due to gearbox backlash / tire compliance / wheels in air, the motor rotor
// sees motor inertia Jm at ~33 Hz (w ~ 200 rad/s), where |G(jw)| ~ 19 (rad/s)/V.
// Stability requires effective Kp_eff = Kp_vel * (1 + Kp_turn) < 1/19 = 0.05!
const float Kp_vel = 0.02f;           // Proportional gain (V / (rad/s)) - rock-solid stability
const float taui_vel = 0.04f;         // Integral time constant (s) - fast integral tracking (Ki = 0.5 V/rad)
const float maxMotorVolt = 9.0f;      // Saturation limit (V)
float velErr_prev[2] = {0.0f, 0.0f};  // e_vel(k-1) for [0]=left, [1]=right
float velInt[2] = {0.0f, 0.0f};       // u_I(k-1) for [0]=left, [1]=right
float inVelLeft = 0.0f;               // Reference velocity left wheel (m/s)
float inVelRight = 0.0f;              // Reference velocity right wheel (m/s)

// Outer Loop: Turnrate & Heading Controller Parameters (Week 7 & Week 8)
// MISSION_MODE: 0 = Straight 1m (Slide 11), 1 = Swerve test (Slide 2), 2 = Square 0.75m at 0.75m/s (Slide 7)
#define MISSION_MODE 2
bool useHeadingControl = true;        // true = Week 8 Heading controller (desiredHeading in rad)
bool useTurnrateControl = true;       // true = inner turnrate damping/PI active
bool useImuTurnrate = true;           // true = use IMU gyro (gyroTurnrate & gyroHeading), false = encoder
const float Kp_head = 5.0f;           // Heading proportional gain Kph ((rad/s) / rad)
const float maxTurnrate = 5.0f;       // Max commanded turnrate (rad/s) for 90-deg turns at 0.75 m/s
const float Kp_turn = 0.5f;           // Turnrate proportional gain Kpt
const float taui_turn = 0.15f;        // Turnrate integral time constant taui_t (s)
float turnErr_prev = 0.0f;            // e_turn(k-1)
float turnInt = 0.0f;                 // tr_i(k-1)
float headU = 0.0f;                   // Velocity difference u = tr_u * (B/2) (m/s)
float stepStartDist = 0.0f;           // Helper to measure distance per segment in multi-step missions
int squareSide = 0;                   // Counter for 4 turns of the square mission
int logTickCnt = 0;                   // Decimation counter so ~6.5s square mission fits in RAM log

// ////////////////////////////////////////
// define a structure for log elements
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
  float accVel;
  float accTurn;
} LogData;
//
// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries

void printLog()
{
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  // hard-coded settings
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, PWM ");  Serial.print(experimentPWMfrq);
  Serial.print(" Hz, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  // Columns
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

  if (logsCnt == 0)
  {
    Serial.println("% WARNING: Log is empty (0 samples). Type 'start' to run the sequence first!");
  }

  for (int i = 0; i < logsCnt; i++)
  {
    // Serial.print(" ");
    Serial.print(d->time,4);    // 1
    Serial.print(" ");    Serial.print(d->state);          // 2
    Serial.print(" ");    Serial.print(d->desiredValue,2); // 3
    Serial.print(" ");    Serial.print(d->motorVolt[0],2); // 4 (V)
    Serial.print(" ");    Serial.print(d->motorVolt[1],2); // 5 (V)
    Serial.print(" ");    Serial.print(d->encoder[0]);     // 6
    Serial.print(" ");    Serial.print(d->encoder[1]);     // 7
    Serial.print(" ");    Serial.print(d->motorVel[0],2);  // 8 (rad/s)
    Serial.print(" ");    Serial.print(d->motorVel[1],2);  // 9 (rad/s)
    Serial.print(" ");    Serial.print(d->battery,2);      // 10 (V)
    Serial.print(" ");    Serial.print(d->gyro[0],2);      // 11 (deg/s)
    Serial.print(" ");    Serial.print(d->gyro[1],2);      // 12 (deg/s)
    Serial.print(" ");    Serial.print(d->gyro[2],2);      // 13 (deg/s)
    Serial.print(" ");    Serial.print(d->motorCurrent[0],4);  // 14 (A)
    Serial.print(" ");    Serial.print(d->motorCurrent[1],4);  // 15 (A)
    Serial.print(" ");    Serial.print(d->poseVel[0],4);       // 16 v (m/s)
    Serial.print(" ");    Serial.print(d->poseVel[1],4);       // 17 turnrate enc (rad/s)
    Serial.print(" ");    Serial.print(d->distA,4);            // 18 distance (m)
    Serial.print(" ");    Serial.print(d->pose[2],4);          // 19 heading enc (rad)
    Serial.print(" ");    Serial.print(d->pose[0],4);          // 20 x (m)
    Serial.print(" ");    Serial.print(d->pose[1],4);          // 21 y (m)
    Serial.print(" ");    Serial.print(d->headU,4);            // 22 headU (m/s)
    Serial.print(" ");    Serial.print(d->accTurn,4);          // 23 gyroTurnrate (rad/s)
    Serial.print(" ");    Serial.print(d->pose[3],4);          // 24 gyroHeading (rad)
    Serial.print(" ");    Serial.print(d->desiredHeading,4);   // 25 desiredHeading (rad)
    Serial.print(" ");    Serial.print(d->encDelay[0]*1000,2); // 26 (ms)
    Serial.print(" ");    Serial.print(d->encDelay[1]*1000,2); // 27 (ms)
    Serial.println("");
    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// ////////////////////////////////////////

/**
 * Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}
/**
 * declaration, that there is a reset function */
void reset();

/**
 * Global variables for state machine */
int state = 0;               // actual sequence state
float endTime = 0;           // for current state
float desiredValue = 0;      // desired forward velocity reference (m/s)
float desiredHeading = 0;    // desired turnrate / heading reference

/**
 * Reset all integrators and memory variables before a new run */
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

  gyroZ_offset = imu2.gyro[2];
  gyroTurnrate = 0.0f;
  gyroTurnrate_prev = 0.0f;
  gyroHeading = 0.0f;

  velErr_prev[0] = 0.0f;
  velErr_prev[1] = 0.0f;
  velInt[0] = 0.0f;
  velInt[1] = 0.0f;
  inVelLeft = 0.0f;
  inVelRight = 0.0f;

  turnErr_prev = 0.0f;
  turnInt = 0.0f;
  headU = 0.0f;
  velSaturated = false;
  stepStartDist = 0.0f;
  squareSide = 0;
  logTickCnt = 0;
}

void start()
{ // Start timing
  // reset log, encoders, pose and controller integrators
  logsCnt = 0;
  reset();
  // Set experiment PWM frequency
  motor.setPWMfrq(experimentPWMfrq);
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;
}

void stop(float after)
{ // stop, but allow continued logging for a while
  state = 2;
  desiredValue = 0;
  // Keep desiredHeading at its final target angle during braking
  // end log after an additional short time
  endTime = time_sec + after;
}

void finished()
{ // prepare for a new start
  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0;  // right motor
  // set default PWM frequency
  motor.setPWMfrq(77824);
  // start updating display
  display.useDisplay = true;
  // wait for next button press
  state = 0;
  printLog();
}

/**
 * ==============================================================
 * SCHRITT 2: Odometrie & Pose-Berechnung mit Tustin-Integratoren 
 * ==============================================================
 * Formel fuer jeden 1/s Integratorblock:
 *   y = T/2 * (x + x1) + y1;
 *   x1 = x;
 *   y1 = y;
 */
void updatePose()
{
  float T = sampleTimeUs * 1e-6f; // Sample time in seconds (0.001 s)

  // 1) Umrechnung Motor-Winkelgeschwindigkeit (rad/s) -> Radgeschwindigkeit (m/s)
  // Achtung: Linker Motor [0] ist umgekehrt eingebaut, daher Minuszeichen!
  float vLeft  = (-encoder.motorVelocity[0] / gear) * wheelRadius;
  float vRight = ( encoder.motorVelocity[1] / gear) * wheelRadius;

  // 2) Vorwaertsgeschwindigkeit v (m/s) und Drehrate turnrate (rad/s)
  poseVel[0] = 0.5f * (vRight + vLeft);       // v = (vR + vL) / 2
  poseVel[1] = (vRight - vLeft) / wheelBase;  // h_dot = (vR - vL) / B

  // 3) Tustin-Integration fuer Distanz distA (m) und Heading pose[2] (rad)
  distA   += (T * 0.5f) * (poseVel[0] + poseVel_prev[0]);
  pose[2] += (T * 0.5f) * (poseVel[1] + poseVel_prev[1]);

  // Aktuelle Eingangs-Werte fuer den naechsten Abtastschritt speichern (x1 = x)
  poseVel_prev[0] = poseVel[0];
  poseVel_prev[1] = poseVel[1];

  // 4) Zerlegung in Weltkoordinaten (x_dot, y_dot) und Tustin-Integration fuer x, y
  float vx = poseVel[0] * cosf(pose[2]);
  float vy = poseVel[0] * sinf(pose[2]);

  pose[0] += (T * 0.5f) * (vx + vx_prev); // x-Position (m)
  pose[1] += (T * 0.5f) * (vy + vy_prev); // y-Position (m)

  vx_prev = vx;
  vy_prev = vy;

  // 5) Week 8 (Step 1): Gyroskop-Drehrate (rad/s) mit 22-Grad-Neigungskorrektur & Tustin-Heading
  if (state == 10)
  { 
    gyroZ_offset = 0.95f * gyroZ_offset + 0.05f * imu2.gyro[2];
    gyroTurnrate = 0.0f;
    gyroTurnrate_prev = 0.0f;
    gyroHeading = 0.0f;
  }
  else
  {
    // Umrechnung deg/s -> rad/s und Division durch cos(22 deg) wegen schraeg eingebauter IMU
    gyroTurnrate = ((imu2.gyro[2] - gyroZ_offset) * (float(M_PI) / 180.0f)) / cosf(imuTiltRad);
    // Tustin-Integration fuer Gyro-Heading h_gyro (rad)
    gyroHeading += (T * 0.5f) * (gyroTurnrate + gyroTurnrate_prev);
    gyroTurnrate_prev = gyroTurnrate;
  }
  pose[3] = gyroHeading;
}

/**
 * 3 x 6V spikes, 10ms each, 300ms apart. Wheels off the floor. */
void spikeSequence()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  { // run mission, initial value
    case 0: // State 0 is just inactive, waiting for the start signal.
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { // starting by a short time to get zero velocity data to the log
        // reset for new run
        start();
        // Prepare next state
        desiredValue = 0; // reference value to the controller
        // to get start the log with no velocity
        endTime = time_sec + 0.30; // new state to end after 20ms
        state = 10;
      }
      break;
    case 10: // Waiting for first step (zero velocity to log) to finish.
      if (time_sec > endTime)
      { // change to next values - drive
        desiredValue = 6;  // should be in meters/sec, but is motor voltage for now,
        endTime = time_sec + 0.010 ; // ~ 0.01 second
        state = 11;
      }
      break;
    case 11: // First step
      // test if ready for next state
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 0;
        endTime = time_sec + 0.30;
        state = 12;
      }
      break;
    case 12: // Second step
      // test if this state is finished
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 6;
        endTime = time_sec + 0.01;
        state = 13;
      }
      break;

    case 13: // First step
      // test if ready for next state
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 0;
        endTime = time_sec + 0.30;
        state = 14;
      }
      break;
    
    case 14: // third step
      // test if this state is finished
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 6;
        endTime = time_sec + 0.01;
        state = 100;
      }
      break;

    
    case 100: // Maintain last step
      // stop after end time
      if (time_sec > endTime)
      { // stop, but continue logging for a while (0.3 sec)
        stop(0.3); // actually sets state to 999 (i.e. default:)
      }
      break;
    default: // Hold until finished
      // this state is needed to enable a new start
      if (time_sec > endTime)
        finished();
      break;
  }
}


/**
 * =========================================================================
 * SCHRITT 4: Fahrsequenzen fuer Week 8 (MISSION_MODE 0, 1 oder 2)
 *   MISSION_MODE 0 (Folie 11): 3.0s geradeaus mit 0.33 m/s (Heading = 0 rad)
 *   MISSION_MODE 1 (Folie 2): Schlenker-Test (0 rad -> -0.3 rad -> 0 rad)
 *   MISSION_MODE 2 (Default, Folie 7): Quadrat fahren (0.75m + 4 x 90 deg & 0.75m)
 * ========================================================================= */
void SequenceTwoSteps()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  switch (state)
  {
    case 0: // State 0: wait for start button press or USB 'start' command
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      {
        start();
        desiredValue = 0.0f;   // 0 m/s
        desiredHeading = 0.0f; // 0 rad
        endTime = time_sec + 0.100f; // 100ms standstill for gyro offset calibration
        state = 10;
      }
      break;

#if MISSION_MODE == 0
    // --- MODE 0 (Folie 11): Straight drive with Heading & Gyro Control ---
    case 10:
      if (time_sec > endTime)
      {
        desiredValue = 0.33f;   // 0.33 m/s forward velocity (~1.0 m in 3s)
        desiredHeading = 0.0f;  // 0.0 rad target heading (straight ahead)
        endTime = time_sec + 3.0f;
        state = 100;
      }
      break;
    case 100:
      if (time_sec > endTime)
      {
        stop(0.3f); // Brake for 0.3s while keeping logging active
      }
      break;

#elif MISSION_MODE == 1
    // --- MODE 1 (Folie 2): Swerve test (0 rad -> -0.3 rad -> 0 rad) ---
    case 10:
      if (time_sec > endTime)
      {
        desiredValue = 0.40f;   // 0.40 m/s
        desiredHeading = 0.0f;  // Start straight (0 rad)
        stepStartDist = distA;
        state = 11;
      }
      break;
    case 11: // Drive 0.40 m straight, then command heading -0.3 rad
      if ((distA - stepStartDist) >= 0.40f)
      {
        desiredHeading = -0.30f; // Turn to -0.3 rad
        stepStartDist = distA;
        state = 12;
      }
      break;
    case 12: // Drive 0.40 m at -0.3 rad, then return to 0.0 rad
      if ((distA - stepStartDist) >= 0.40f)
      {
        desiredHeading = 0.0f;   // Return to 0.0 rad
        stepStartDist = distA;
        state = 100;
      }
      break;
    case 100: // Final 0.50 m straight at 0.0 rad
      if ((distA - stepStartDist) >= 0.50f || time_sec > 3.2f)
      {
        stop(0.3f);
      }
      break;

#elif MISSION_MODE == 2
    //   1) Fwd 0.75 m at 0.75 m/s (desiredHeading = 0 rad)
    //   2) 4 x: Turn 90 deg (+pi/2 rad) and Fwd 0.75 m at 0.75 m/s
    case 10:
      if (time_sec > endTime)
      {
        desiredValue = 0.75f;   // 0.75 m/s forward speed
        desiredHeading = 0.0f;  // Initial leg: 0 rad (0 deg)
        stepStartDist = distA;
        squareSide = 0;
        state = 11;
      }
      break;
    case 11:
      // Check if robot has traveled 0.75 m on the current side
      if ((distA - stepStartDist) >= 0.75f)
      {
        if (squareSide < 3)
        {
          // 4 x: Turn 90 deg (+pi/2 rad) and drive another 0.75 m forward
          squareSide++;
          desiredHeading = squareSide * (float(M_PI) * 0.5f); // 90, 180, 270, 360 deg
          stepStartDist = distA;
          state = 11 + squareSide; // state 12, 13, 14, 15 (visible in log like on Slide 7!)
        }
        else
        {
          // Finished initial 0.75m + 4 x 0.75m -> stop and log braking for 0.8s
          stop(0.8f);
        }
      }
      break;
    case 12:
    case 13:
    case 14:
    case 15:
      if ((distA - stepStartDist) >= 0.75f)
      {
        if (squareSide < 3)
        {
          squareSide++;
          desiredHeading = squareSide * (float(M_PI) * 0.5f);
          stepStartDist = distA;
          state = 11 + squareSide;
        }
        else
        {
          stop(0.8f);
        }
      }
      break;
#endif

    default: // Hold until finished
      if (time_sec > endTime)
        finished();
      break;
  }
}

/**
 * =========================================================================
 * SCHRITT 3: Kaskadenregelung in controlUpdate() (Week 7 & Week 8)
 *   1) Aeusserer Heading-P-Regler (Week 8, Folie 4-6): Winkel h -> Soll-Drehrate
 *   2) Mittlerer Turnrate-PI-Regler (Week 7/8): Drehrate -> Geschwindigkeitsdifferenz headU
 *   3) Innere Motor-Geschwindigkeits-PI-Regler: Radgeschwindigkeiten -> Motorspannung
 * ========================================================================= */
void controlUpdate()
{ // do control during a mission only.
  // called at every tick
  float T = sampleTimeUs * 1e-6f; // Sample time Ts in seconds (0.001 s)

  float velRef = desiredValue;    // Desired forward velocity v_ref (m/s)
  float turnRef = 0.0f;           // Desired turnrate h_dot_ref (rad/s)

#if USE_P_CONTROL
  // Plain P-controller on each motor in motor rad/s (lecture 5), no turnrate loop
  float wP = velRef * (gear / wheelRadius);
  float uL = constrain(Kp_step * (wP + encoder.motorVelocity[0]), -maxMotorVolt, maxMotorVolt);
  float uR = constrain(Kp_step * (wP - encoder.motorVelocity[1]), -maxMotorVolt, maxMotorVolt);
  motor.motorVoltage[0] = -uL; // Left motor runs backwards
  motor.motorVoltage[1] =  uR;
  (void)T; (void)turnRef;
  return;
#endif

  // -----------------------------------------------------------------------
  // TEIL A1 (Week 8, Folie 4-6): Aeusserer Heading-Regler (Winkelregelung)
  // -----------------------------------------------------------------------
  if (useHeadingControl)
  {
    // Auswahl des Winkel-Sensors: Integriertes Gyro-Heading (gyroHeading) oder Encoder (pose[2])
    float headMeas = useImuTurnrate ? gyroHeading : pose[2];

    // 1. Winkelfehler e_head = h_ref - h_meas (in rad)
    float e_head = desiredHeading - headMeas;

    // 2. Proportional-Regler Kph: berechnet gewuenschte Drehrate u_head (in rad/s)
    turnRef = Kp_head * e_head;

    // 3. Drehraten-Begrenzung (verhindert Radschlupf bei grossen 90-Grad-Spruengen, Folie 10!)
    if (turnRef >  maxTurnrate) turnRef =  maxTurnrate;
    if (turnRef < -maxTurnrate) turnRef = -maxTurnrate;
  }
  else
  {
    // Wenn Heading-Regler aus ist (Woche 7 Modus), wird desiredHeading direkt als Soll-Drehrate (rad/s) genutzt
    turnRef = desiredHeading;
  }

  // -----------------------------------------------------------------------
  // TEIL A2 (Week 7/8): Turnrate-PI-Regler / Mixer (Drehrate -> Rad-Differenz)
  // -----------------------------------------------------------------------
  if (useTurnrateControl)
  {
    // Auswahl des Drehraten-Sensors: neigungskorrigiertes IMU Gyro Z (gyroTurnrate) oder Encoder (poseVel[1])
    float turnMeas = useImuTurnrate ? gyroTurnrate : poseVel[1];

    // 1. Regelfehler e_turn = h_dot_ref - h_dot_meas
    float e_turn = turnRef - turnMeas;

    // 2. P-Anteil: kp = Kpt * e_turn
    float up_turn = Kp_turn * e_turn;

    // 3. I-Anteil nach Tustin: tr_i(k) = tr_i(k-1) + (Kpt * Ts)/(2 * taui_t) * (e(k) + e(k-1))
    // Anti-Windup: Nur weiter integrieren, wenn die Motoren nicht in der 9V-Saettigung sind!
    if (!velSaturated)
    {
      turnInt += (Kp_turn * T / (2.0f * taui_turn)) * (e_turn + turnErr_prev);
    }
    turnErr_prev = e_turn;

    // 4. Reglerausgang tr_u = kp + tr_i und Umrechnung mit B/2 im Mixer (m/s)
    // Zusammen mit der Vorsteuerung turnRef ergibt sich u_head + Korrektur:
    float tr_u = turnRef + up_turn + turnInt;
    headU = tr_u * (wheelBase * 0.5f);
  }
  else
  {
    // Direkter Mixer wie auf Folie 4 & 5: headU = u_head * (B/2)
    headU = turnRef * (wheelBase * 0.5f);
  }

  // Aufteilung auf linkes und rechtes Rad (Inverse Kinematik)
  inVelLeft  = velRef - headU; // (m/s)
  inVelRight = velRef + headU; // (m/s)

  // -----------------------------------------------------------------------
  // TEIL B: Innere Motor-PI-Regler fuer linkes [0] und rechtes [1] Rad
  // -----------------------------------------------------------------------
  // Umrechnung Rad-Sollgeschwindigkeit (m/s) -> Motorwellen-Drehzahl (rad/s)
  float wRef[2];
  wRef[0] = inVelLeft  * (gear / wheelRadius);
  wRef[1] = inVelRight * (gear / wheelRadius);

  // Gemessene Motorwellen-Drehzahl (rad/s)
  float wMeas[2];
  wMeas[0] = -encoder.motorVelocity[0]; // Linker Motor laeuft rueckwaerts -> Vorzeichen drehen
  wMeas[1] =  encoder.motorVelocity[1];

  float uVolt[2] = {0.0f, 0.0f};
  velSaturated = false; // Wird true, sobald mindestens ein Motor +/- 9V erreicht

  for (int m = 0; m < 2; m++)
  {
    // 1. Regelfehler an der Motorwelle (rad/s)
    float e_vel = wRef[m] - wMeas[m];

    // 2. P-Anteil: u_P(k) = Kp * e(k)
    float u_p = Kp_vel * e_vel;

    // 3. Tustin-Zuwachs fuer I-Anteil: u_I(k) = u_I(k-1) + (Kp * Ts)/(2 * tau_i) * (e(k) + e(k-1))
    velInt[m] += (Kp_vel * T / (2.0f * taui_vel)) * (e_vel + velErr_prev[m]);

    // Anti-Windup (Integrator Clamping auf +/- 9V, genau wie in motorPI.slx)
    if (velInt[m] > maxMotorVolt)  velInt[m] = maxMotorVolt;
    if (velInt[m] < -maxMotorVolt) velInt[m] = -maxMotorVolt;

    // 4. Gesamte Motorspannung u = u_P + u_I und Stellgroessenbegrenzung (+/- 9V)
    float u_total = u_p + velInt[m];
    if (u_total > maxMotorVolt)
    {
      u_total = maxMotorVolt;
      velSaturated = true;
    }
    else if (u_total < -maxMotorVolt)
    {
      u_total = -maxMotorVolt;
      velSaturated = true;
    }

    // Fehler fuer naechsten Abtastschritt speichern
    velErr_prev[m] = e_vel;
    uVolt[m] = u_total;
  }

  // Motorspannungen an Hardware-Treiber uebergeben (-9V bis +9V)
  motor.motorVoltage[0] = -uVolt[0]; // Linker Motor braucht umgekehrtes Vorzeichen
  motor.motorVoltage[1] =  uVolt[1]; // Rechter Motor
}

void updateLog()
{ // step response mode
  if (state > 0 and logsCnt < logsMax)
  {
    logTickCnt++;
#if MISSION_MODE == 2
    // Square mission takes ~6.2s -> log every 2nd sample (2ms = 500Hz) so full 6.6s fits in RAM!
    if ((logTickCnt % 2) != 1) return;
#endif
    // save data for this sample into dataLog structure
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; // The left motor runs backwards, therefore the sign.
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Tania: (+0.260A, -0.275A)
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
    logs[logsCnt].accTurn = gyroTurnrate;
    // add more items as needed, NB! also the the struct definition at top.
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
    logs[logsCnt].encDelay[0] = encoder.encoderDelay[0];
    logs[logsCnt].encDelay[1] = encoder.encoderDelay[1];
    logsCnt++;
  }
}


void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  encoder.pulsPerRev = 48;
  imu2.setup(); // gyro and accelerometer
  motor.setPWMfrq(0x13000); // 0x13000 = 77824 Hz
  usb.setup();
  display.useDisplay = true;
}

/**
* Main loop
* */
void loop ( void )
{ // init sample time
  while ( true )
  { // main loop
    // loop until time for next sample
    if (isSampleTime) // start of new control cycle
    { // time for next tick
      isSampleTime = false;
      // read sensors
      imu2.tick();
      encoder.tick();
      updatePose();
#if USE_SPIKE_SEQUENCE
      spikeSequence();
#else
      SequenceTwoSteps();
#endif
      //
      if (state > 0)
      { // Only if started
        // Calculate new motor voltage
        controlUpdate();
      }
      // give value to actuators
      motor.tick();
      // save relevant values
      updateLog();
      // support functions
      robot.tick();   // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}

/////////////////////////////////////////////////////////////////
