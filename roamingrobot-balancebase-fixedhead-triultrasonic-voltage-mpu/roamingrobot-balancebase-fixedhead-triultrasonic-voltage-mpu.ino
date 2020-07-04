#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

//Pin Configuration
const int TRIGGER_PIN = 7; // Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN_CENTER = 6; // Arduino pin tied to echo pin on ping sensor.
const int ECHO_PIN_LEFT = 8; // Arduino pin tied to echo pin on ping sensor.
const int ECHO_PIN_RIGHT = 9; // Arduino pin tied to echo pin on ping sensor.

//functional Configuration
const int avoidableObstacleRange = 50;//cm
const int emergencyObstacleRange = 25; //cm
const int timeToStickRightLeftDecission = 12000;//milli seconds
const int backMovementTime = 2000;//milli seconds
const int rightLeftMovementTime = 3000;//milli seconds
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 5;                                    //Turning speed (20)
float max_target_speed = 2;                                //Max target speed (100)
int acc_calibration_value = 675;                            //Enter the accelerometer calibration value
int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)

//Dont touch below stuff
byte start, received_byte, low_bat;
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value;
unsigned long loop_timer;
float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

unsigned long overrideForwardUntill, overrideRightUntill, overrideLeftUntill, lastRightLeftDecidedTime, lastSonarSentTime;
volatile unsigned long lastEchoReceivedTimeCenter, lastEchoReceivedTimeLeft, lastEchoReceivedTimeRight;
volatile int nextCheckEcho = -1;
boolean isRightDecidedCached = false;
float centerReading = 0.0;
float leftReading = 0.0;
float rightReading = 0.0;

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN_CENTER, INPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  Serial.begin(115200);
  setupBase();
  lastSonarSentTime = micros();
  lastEchoReceivedTimeCenter = micros() + 10000;
  lastEchoReceivedTimeLeft = micros() + 10000;
  lastEchoReceivedTimeRight = micros() + 10000;
  overrideForwardUntill = millis();
  overrideRightUntill = millis();
  overrideLeftUntill = millis();
  doBIOSManoeuvre();
}

void loop() {
  populateUltrasonicReading();
  Serial.println(centerReading);

  //incase of emergency
  if (isObstacleWithinEmergencyDistance()) {
    markForForwardOverride(backMovementTime);
    if (!isForwardOverridden()) {
      if (isRightDecided()) {
        markForRightOverride(backMovementTime+rightLeftMovementTime);
      } else {
        markForLeftOverride(backMovementTime+rightLeftMovementTime);
      }
    }
  }

  //Only do this when you are moving forward
  else if (isObstacleWithinAvoidableDistance()) {
    //below is true means im going forward
    if (!isForwardOverridden()) {
      if (isRightDecided()) {
        markForRightOverride(rightLeftMovementTime);
      } else {
        markForLeftOverride(rightLeftMovementTime);
      }
    }
  }

  rotateOrSteerAndGoTowardsDestination();

  loopBase();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while (loop_timer > micros());
  loop_timer += 4000;

}

void markForForwardOverride(unsigned long overrideTime) {
  if (overrideForwardUntill < millis()) {
    overrideForwardUntill = millis() + overrideTime;
  }
}

void markForRightOverride(unsigned long overrideTime) {
  if (overrideRightUntill < millis()) {
    overrideRightUntill = millis() + overrideTime;
  }
}

void markForLeftOverride(unsigned long overrideTime) {
  if (overrideLeftUntill < millis()) {
    overrideLeftUntill = millis() + overrideTime;
  }
}

boolean isForwardOverridden() {
  return overrideForwardUntill > millis();
}

boolean isRightOverridden() {
  return overrideRightUntill > millis();
}

boolean isLeftOverridden() {
  return overrideLeftUntill > millis();
}

boolean isObstacleWithinAvoidableDistance() {
  return (centerReading > emergencyObstacleRange && centerReading <= avoidableObstacleRange)
         || (leftReading > emergencyObstacleRange/2 && leftReading <= avoidableObstacleRange/2)
         || (rightReading > emergencyObstacleRange/2 && rightReading <= avoidableObstacleRange/2);
}

boolean isObstacleWithinEmergencyDistance() {
  return (centerReading > 0 && centerReading <= emergencyObstacleRange)
         || (leftReading > 0 && leftReading <= emergencyObstacleRange/2)
         || (rightReading > 0 && rightReading <= emergencyObstacleRange/2);
}

boolean isRightDecided() {
  if (millis() - lastRightLeftDecidedTime > timeToStickRightLeftDecission) {
    isRightDecidedCached = (random(0, 2) < 1);
    lastRightLeftDecidedTime = millis();
  }
  return isRightDecidedCached;
}

void doBIOSManoeuvre() {
  unsigned long botStartTime = millis();
  while (millis() - botStartTime < 10000) {
    baseStop();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //left
  while (millis() - botStartTime < 15000) {
    baseRotateLeft();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //right
  while (millis() - botStartTime < 20000) {
    baseRotateRight();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //forward
  while (millis() - botStartTime < 25000) {
    baseGoForward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //backward
  while (millis() - botStartTime < 30000) {
    baseGoBackward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

}

void rotateOrSteerAndGoTowardsDestination() {
  if (isForwardOverridden()) {
    baseGoBackward();
  } else if (isRightOverridden()) {
    baseRotateRight();
  } else if (isLeftOverridden()) {
    baseRotateLeft();
  } else {
    baseGoForward();
  }
}

void populateUltrasonicReading() {
  centerReading = (lastEchoReceivedTimeCenter - lastSonarSentTime) / 58.2;
  leftReading = (lastEchoReceivedTimeLeft - lastSonarSentTime) / 58.2;
  rightReading = (lastEchoReceivedTimeRight - lastSonarSentTime) / 58.2;
  //Serial.println(obstacleDist);
  if (micros() - lastSonarSentTime > 250000) {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    lastSonarSentTime = micros();
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(17);
    digitalWrite(TRIGGER_PIN, LOW);
  }
}

//Within interrupt. Should run fast.
void triggerSonarInterrupt() {
  if (nextCheckEcho == -1) {
    if (digitalRead(ECHO_PIN_LEFT) == HIGH)
      lastEchoReceivedTimeLeft = micros();
    nextCheckEcho = 0;
  }else if (nextCheckEcho == 0) {
    if (digitalRead(ECHO_PIN_CENTER) == HIGH)
      lastEchoReceivedTimeCenter = micros();
    nextCheckEcho = 1;
  }else if (nextCheckEcho == 1) {
    if (digitalRead(ECHO_PIN_RIGHT) == HIGH)
      lastEchoReceivedTimeRight = micros();
    nextCheckEcho = -1;
  }
}
