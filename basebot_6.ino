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
// 1 = spikeSequence (transient, R/L), 0 = SequenceTwoSteps (Driving straight test)
#define USE_SPIKE_SEQUENCE 0
// Sample time can not go lower than 300us (1000us = 1ms allows ~4.3s of logging)
const uint32_t sampleTimeUs = 1000;   // Ts = 1 ms = 0.001 s
// Default high PWM frequency (77824 Hz) for quiet, ripple-free operation
const uint32_t experimentPWMfrq = 77824;

// Robot geometry configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.14;     // (m) distance B between left and right wheel

// Pose and kinematic state values
float distA = 0;     // trip distance (m)
float pose[4];       // [0]: x (m), [1]: y (m), [2]: heading h (rad), [3]: tilt (rad)
float poseVel[2];    // [0]: forward velocity v (m/s), [1]: turnrate h_dot (rad/s)
bool velSaturated = false;

// Memory variables (x1 = value from last sample k-1) for Tustin integrators in updatePose()
float poseVel_prev[2] = {0.0f, 0.0f}; // [0]: v(k-1), [1]: turnrate(k-1)
float vx_prev = 0.0f;                 // vx(k-1) = v(k-1)*cos(h(k-1))
float vy_prev = 0.0f;                 // vy(k-1) = v(k-1)*sin(h(k-1))

// Inner Loop: Motor Velocity PI Controller Parameters (from piDesign.mat)
const float Kp_vel = 0.2f;            // Proportional gain (V / (rad/s))
const float taui_vel = 1.0f;          // Integral time constant (s)
const float maxMotorVolt = 9.0f;      // Saturation limit (V)
float velErr_prev[2] = {0.0f, 0.0f};  // e_vel(k-1) for [0]=left, [1]=right
float velInt[2] = {0.0f, 0.0f};       // u_I(k-1) for [0]=left, [1]=right
float inVelLeft = 0.0f;               // Reference velocity left wheel (m/s)
float inVelRight = 0.0f;              // Reference velocity right wheel (m/s)

// Outer Loop: Turnrate PI Controller Parameters (Week 7, Slide 13 & 14)
bool useTurnrateControl = true;       // true = closed-loop turnrate PI active
bool useImuTurnrate = false;          // false = encoder turnrate poseVel[1], true = IMU gyro Z
const float Kp_turn = 5.0f;           // Turnrate proportional gain Kpt
const float taui_turn = 0.2f;         // Turnrate integral time constant taui_t (s)
float turnErr_prev = 0.0f;            // e_turn(k-1)
float turnInt = 0.0f;                 // tr_i(k-1)
float headU = 0.0f;                   // Velocity difference u = tr_u * (B/2) (m/s)

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
  Serial.println("% 19  Heading pose[2] (rad)");
  Serial.println("% 20-21 Position x, y (pose[0], pose[1]) (m)");
  Serial.println("% 22  Turnrate controller output headU (m/s)");

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
    Serial.print(" ");    Serial.print(d->poseVel[1],4);       // 17 turnrate (rad/s)
    Serial.print(" ");    Serial.print(d->distA,4);            // 18 distance (m)
    Serial.print(" ");    Serial.print(d->pose[2],4);          // 19 heading (rad)
    Serial.print(" ");    Serial.print(d->pose[0],4);          // 20 x (m)
    Serial.print(" ");    Serial.print(d->pose[1],4);          // 21 y (m)
    Serial.print(" ");    Serial.print(d->headU,4);            // 22 headU (m/s)
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
float desiredHeading = 0;    // desired turnrate reference (rad/s)

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
  desiredHeading = 0;
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
 * =========================================================================
 * SCHRITT 2: Odometrie & Pose-Berechnung mit Tustin-Integratoren (Folie 9)
 * =========================================================================
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
  pose[3] = imu2.tilt;
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
 * SCHRITT 4: Fahrsequenz fuer "Driving Straight" (3s mit 0.33 m/s = 1 Meter)
 * ========================================================================= */
void SequenceTwoSteps()
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
        desiredValue = 0.0f;   // 0 m/s
        desiredHeading = 0.0f; // 0 rad/s turnrate
        // to get start the log with no velocity
        endTime = time_sec + 0.100f; // 100ms of zero as baseline
        state = 10;
      }
      break;
    case 10: // Waiting for first step (zero velocity to log) to finish.
      if (time_sec > endTime)
      { // Drive straight at 0.33 m/s for 3.0 seconds (~1.0 meter distance)
        desiredValue = 0.33f;   // forward velocity reference in m/s
        desiredHeading = 0.0f;  // turnrate reference in rad/s (0 = straight)
        endTime = time_sec + 3.0f;
        state = 100;
      }
      break;
    case 100: // Maintain step until 3.0 seconds are over
      // stop after end time
      if (time_sec > endTime)
      { // stop, but continue logging for a while (0.3 sec)
        stop(0.3f); // sets desiredValue = 0 and logs braking for 0.3s
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
 * SCHRITT 3: Kaskadenregelung in controlUpdate() (Folie 13-15)
 *   1) Aeusserer Turnrate-PI-Regler (Tustin + Anti-Windup)
 *   2) Innere Motor-Geschwindigkeits-PI-Regler (Tustin + 9V Anti-Windup)
 * ========================================================================= */
void controlUpdate()
{ // do control during a mission only.
  // called at every tick
  float T = sampleTimeUs * 1e-6f; // Sample time Ts in seconds (0.001 s)

  float velRef  = desiredValue;   // Desired forward velocity v_ref (m/s)
  float turnRef = desiredHeading; // Desired turnrate h_dot_ref (rad/s)

  // -----------------------------------------------------------------------
  // TEIL A: Aeusserer Turnrate-PI-Regler (Week 7, Slide 13 & 14)
  // -----------------------------------------------------------------------
  if (useTurnrateControl)
  {
    // Auswahl des Drehraten-Sensors: Encoder (poseVel[1]) oder IMU Gyro Z (in rad/s)
    float turnMeas = useImuTurnrate ? (imu2.gyro[2] * float(M_PI) / 180.0f) : poseVel[1];

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

    // 4. Reglerausgang tr_u = kp + tr_i und Umrechnung mit B/2 in Geschwindigkeitsdifferenz u (m/s)
    float tr_u = up_turn + turnInt;
    headU = tr_u * (wheelBase * 0.5f);
  }
  else
  {
    // Open-Loop Vorsteuerung wie in Woche 6 (ohne Turnrate-Regler)
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

    // 3. Tustin-Zuwachs fuer I-Anteil: du_I = (Kp * Ts)/(2 * tau_i) * (e(k) + e(k-1))
    float du_i = (Kp_vel * T / (2.0f * taui_vel)) * (e_vel + velErr_prev[m]);
    float u_i_candidate = velInt[m] + du_i;

    // 4. Gesamte Motorspannung vor Begrenzung
    float u_total = u_p + u_i_candidate;

    // 5. Spannungsbegrenzung (+/- 9V) & Anti-Windup (Clamping)
    if (u_total > maxMotorVolt)
    {
      u_total = maxMotorVolt;
      velSaturated = true;
      // Nur integrieren, wenn der Fehler hilft, aus der oberen Saettigung herauszukommen
      if (du_i < 0.0f) velInt[m] = u_i_candidate;
    }
    else if (u_total < -maxMotorVolt)
    {
      u_total = -maxMotorVolt;
      velSaturated = true;
      // Nur integrieren, wenn der Fehler hilft, aus der unteren Saettigung herauszukommen
      if (du_i > 0.0f) velInt[m] = u_i_candidate;
    }
    else
    {
      // Nicht gesaettigt -> Integrator normal aktualisieren
      velInt[m] = u_i_candidate;
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
  { // save data for this sample into dataLog structure
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
    // add more items as needed, NB! also the the struct definition at top.
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
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
