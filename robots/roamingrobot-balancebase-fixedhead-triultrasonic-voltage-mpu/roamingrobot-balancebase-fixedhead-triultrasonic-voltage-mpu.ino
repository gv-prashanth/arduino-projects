#include <Wire.h>
#include "soc/gpio_struct.h"

// ESP32-S Pin Configuration

// Ultrasonic Sensor Pins
const int TRIGGER_PIN = 16;
const int ECHO_PIN_CENTER = 17;
const int ECHO_PIN_LEFT = 27;
const int ECHO_PIN_RIGHT = 14;

// Stepper Motor Pins (A4988 STEP+DIR interface)
const int LEFT_STEP_PIN = 26;
const int LEFT_DIR_PIN = 25;
const int RIGHT_STEP_PIN = 33;
const int RIGHT_DIR_PIN = 32;

// I2C Pins (MPU-6050)
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// LED Pin (built-in on most ESP32 dev boards)
const int LED_PIN = 2;

// Battery ADC Pin (ADC1_CH0, input-only, safe when WiFi active)
const int BATTERY_ADC_PIN = 36;

// DIAGNOSTIC: set to false to disable motors so the angle estimate can be
// verified by tilting the robot by hand without motor vibration/runaway.
#define MOTORS_ENABLED true

//functional Configuration
const int emergencyObstacleRange = 60; //cm
const int timeToStickRightLeftDecission = 2000;//milli seconds
const int backMovementTime = 1000;//milli seconds
const int rightLeftMovementTime = 2000;//milli seconds
float pid_p_gain = 15;                                       //Gain setting for the P-controller -- TUNE THIS FIRST (I and D = 0)
float pid_i_gain = 0;                                        //Gain setting for the I-controller -- TUNE LAST (start 0)
float pid_d_gain = 0;                                        //Gain setting for the D-controller -- TUNE SECOND (start 0)
float turning_speed = 5;                                    //Turning speed (20)
float max_target_speed = 0;                                //Max target speed -- 0 = balance on the spot (no travel) during gain tuning
int acc_calibration_value = -276;                           //Accelerometer calibration: teeter point read ~0.25deg with -240, shifted ~36 counts so balance point reads ~0deg
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
  // doBIOSManoeuvre();  // Disabled for balance debugging
}

void loop() {
  // Ultrasonic disabled for balance debugging — hardcode safe distances
  centerReading = 999.0;
  leftReading = 999.0;
  rightReading = 999.0;

  // Throttle debug prints so Serial output doesn't stretch the 4ms loop.
  // Printing every loop at 115200 baud takes ~5-6ms and corrupts gyro
  // integration timing + PID. Print roughly every 25 loops (~100ms).
  static int printCounter = 0;
  if (++printCounter >= 25) {
    printCounter = 0;
    Serial.print("start="); Serial.print(start);
    Serial.print(" angle="); Serial.print(angle_gyro, 2);
    Serial.print(" acc_angle="); Serial.print(angle_acc, 2);
    Serial.print(" gyro_rate="); Serial.print(gyro_pitch_data_raw);
    Serial.print(" pid="); Serial.print(pid_output, 2);
    Serial.print(" L="); Serial.print(throttle_left_motor);
    Serial.print(" R="); Serial.println(throttle_right_motor);
  }

  baseStop();


  loopBase();

  if (!MOTORS_ENABLED) {
    throttle_left_motor = 0;
    throttle_right_motor = 0;
  }

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

boolean isObstacleWithinEmergencyDistance() {
  return (centerReading > 0 && centerReading <= emergencyObstacleRange)
         || (leftReading > 0 && leftReading <= emergencyObstacleRange / 2)
         || (rightReading > 0 && rightReading <= emergencyObstacleRange / 2);
}

boolean isRightDecided() {
  if (millis() - lastRightLeftDecidedTime > timeToStickRightLeftDecission) {
    isRightDecidedCached = rightReading > leftReading;
    lastRightLeftDecidedTime = millis();
  }
  return isRightDecidedCached;
}

void doBIOSManoeuvre() {
  Serial.println("[BIOS] Phase 1: STOP (10s) - hold robot upright");
  unsigned long botStartTime = millis();
  while (millis() - botStartTime < 10000) {
    baseStop();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }

  Serial.print("[BIOS] Phase 2: LEFT (5s) start="); Serial.println(start);
  //left
  while (millis() - botStartTime < 15000) {
    baseRotateLeft();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }

  Serial.println("[BIOS] Phase 3: RIGHT (5s)");
  //right
  while (millis() - botStartTime < 20000) {
    baseRotateRight();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }

  Serial.println("[BIOS] Phase 4: FORWARD (5s)");
  //forward
  while (millis() - botStartTime < 25000) {
    baseGoForward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }

  Serial.println("[BIOS] Phase 5: BACKWARD (5s)");
  //backward
  while (millis() - botStartTime < 30000) {
    baseGoBackward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }

  Serial.println("[BIOS] Phase 6: FINAL STOP (5s)");
  while (millis() - botStartTime < 35000) {
    baseStop();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
    yield();
  }
  Serial.println("[BIOS] Complete. Entering main loop.");
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

//Within interrupt. Should run fast. IRAM_ATTR required for ESP32 timer ISR context.
void IRAM_ATTR triggerSonarInterrupt() {
  if (nextCheckEcho == -1) {
    if (digitalRead(ECHO_PIN_LEFT) == HIGH)
      lastEchoReceivedTimeLeft = micros();
    nextCheckEcho = 0;
  } else if (nextCheckEcho == 0) {
    if (digitalRead(ECHO_PIN_CENTER) == HIGH)
      lastEchoReceivedTimeCenter = micros();
    nextCheckEcho = 1;
  } else if (nextCheckEcho == 1) {
    if (digitalRead(ECHO_PIN_RIGHT) == HIGH)
      lastEchoReceivedTimeRight = micros();
    nextCheckEcho = -1;
  }
}
