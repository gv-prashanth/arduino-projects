#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include <VoltageSensor.h>

//Pin Configuration
const int batteryVoltageSensePin = -1;//A2 incase you want to detect from dedicated pin. -1 incase you want to detect from vcc.
const int TRIGGER_PIN = 7; // Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 6; // Arduino pin tied to echo pin on ping sensor.

//functional Configuration
const int avoidableObstacleRange = 80;//cm
const int emergencyObstacleRange = 30; //cm
const int timeToStickRightLeftDecission = 12000;//milli seconds
const unsigned long robotJamCheckTime = 120000; //milli seconds
const int baseMovementTime = 1000;//milli seconds
const float sleepVoltage = 3.3;//volts
const float wakeVoltage = 3.7;//volts. Must be greater than sleepVoltage.
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 5;                                    //Turning speed (20)
float max_target_speed = 5;                                //Max target speed (100)
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

unsigned long lastComandedDirectionChangeTime = 0;
unsigned long overrideForwardUntill = 0;
unsigned long lastRightLeftDecidedTime = 0;
boolean isRightDecidedCached = false;
float destinationHeading = 0.0;
float centerReading = 0.0;

unsigned long commandEndTime, botStartTime, lastSonarSentTime, lastEchoReceivedTime;

VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);

void setup() {

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
  setupBase();
  botStartTime = millis();
  lastSonarSentTime = micros();
  lastEchoReceivedTime = micros() + 10000;

  //Wake the robot
  markForWakeup();

  //Print the voltage of battery
  Serial.println("Battery Voltage: " + String(batteryVoltageSensor.senseVoltage()));

  //check if battery is low and skip bios dance
  if (!isBatteryDead()) {
    doBIOSManoeuvre();
  }
}

void loop() {
  populateUltrasonicReading();

  //check if battery is low and go to sleep
  if (isBatteryDead()) {
    markForSleep();
    return;
  }

  //TODO: Need to better check if there is a robot jam
  else if (isJamDetected()) {
    morseCodePlay("Jam");
    markForForwardOverride(baseMovementTime);
    setJamDestination();
  }

  //incase of emergency
  else if (isObstacleWithinEmergencyDistance()) {
    morseCodePlay("Emergency");
    markForForwardOverride(baseMovementTime);
    setEmergencyDestination();
  }

  //Only do this when you are moving forward
  else if (isObstacleWithinAvoidableDistance()) {
    //below is true means im going forward
    if (!isForwardOverridden())
      setAvoidableObstacleDestination();
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

void markForSleep() {
  morseCodePlay("SOS");
}

//TODO: Need to get rid of marks logic
void markForWakeup() {
  morseCodePlay("A");
  lastComandedDirectionChangeTime = millis();
  overrideForwardUntill = millis();
}

void markForForwardOverride(unsigned long overrideTime) {
  if(overrideForwardUntill < millis()){
    overrideForwardUntill = millis() + overrideTime;
  }
}

boolean isForwardOverridden() {
  return overrideForwardUntill > millis();
}

boolean isBatteryCharged() {
  return batteryVoltageSensor.senseVoltage() > wakeVoltage;
}

boolean isJamDetected() {
  return abs(millis() - lastComandedDirectionChangeTime) > robotJamCheckTime;
}

boolean isBatteryDead() {
  return batteryVoltageSensor.senseVoltage() < sleepVoltage;
}

boolean isObstacleWithinAvoidableDistance() {
  return (centerReading > emergencyObstacleRange && centerReading <= avoidableObstacleRange);
}

boolean isObstacleWithinEmergencyDistance() {
  return (centerReading > 0 && centerReading <= emergencyObstacleRange);
}

boolean isRightDecided() {
  if (millis() - lastRightLeftDecidedTime > timeToStickRightLeftDecission) {
    isRightDecidedCached = (random(0, 2) < 1);
    lastRightLeftDecidedTime = millis();
  }
  return isRightDecidedCached;
}

void setEmergencyDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(9, 18) * 10);
  else
    setLeftDestinationByAngle(random(9, 18) * 10);
}

void setJamDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(1, 36) * 10);
  else
    setLeftDestinationByAngle(random(1, 36) * 10);
}

void setAvoidableObstacleDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(1, 36) * 10);
  else
    setLeftDestinationByAngle(random(1, 36) * 10);
}

void doBIOSManoeuvre() {

  while (millis() - botStartTime < 10000) {
    baseStop();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //left
  morseCodePlay("L");
  while (millis() - botStartTime < 15000) {
    baseRotateLeft();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //right
  morseCodePlay("R");
  while (millis() - botStartTime < 20000) {
    baseRotateRight();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //forward
  morseCodePlay("F");
  while (millis() - botStartTime < 22000) {
    baseGoForward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //backward
  morseCodePlay("B");
  while (millis() - botStartTime < 24000) {
    baseGoBackward();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

  //wait
  morseCodePlay("S");
  while (millis() - botStartTime < 30000) {
    baseStop();
    loopBase();
    while (loop_timer > micros());
    loop_timer += 4000;
  }

}

void rotateOrSteerAndGoTowardsDestination() {
  if (isForwardOverridden()) {
    baseGoBackward();
  } else if (calculateAngularDifferenceVector() > 0 && abs(calculateAngularDifferenceVector()) > 1) {
    baseRotateRight();
    setRightDestinationByAngle(abs(calculateAngularDifferenceVector()) - 0.001);
  } else if (calculateAngularDifferenceVector() < 0 && abs(calculateAngularDifferenceVector()) > 1) {
    baseRotateLeft();
    setLeftDestinationByAngle(abs(calculateAngularDifferenceVector()) - 0.001);
  } else {
    destinationHeading = getHeading();
    baseGoForward();
  }
}

float calculateAngularDifferenceVector() {
  //The angular error is calculated by actual - required. Further the easisest / closest direction is choosen as part of returning the value.
  return destinationHeading;
}

void setLeftDestinationByAngle(int angle) {
  float currentHeading = getHeading();
  destinationHeading = currentHeading - angle;
  lastComandedDirectionChangeTime = millis();
}

void setRightDestinationByAngle(int angle) {
  float currentHeading = getHeading();
  destinationHeading = currentHeading + angle;
  lastComandedDirectionChangeTime = millis();
}

void morseCodePlay(String toPlay) {
  Serial.println(toPlay);
}

void baseRotateLeft() {
  received_byte = B00000001;
}

void baseRotateRight() {
  received_byte = B00000010;
}

void baseGoForward() {
  received_byte = B00000100;
}

void baseGoBackward() {
  received_byte = B00001000;
}

void baseStop() {
  received_byte = 0x00;
}

float getHeading() {
  return 0;
}

void populateUltrasonicReading() {
  centerReading = (lastEchoReceivedTime - lastSonarSentTime) / 58.2;
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
  if (digitalRead(ECHO_PIN) == HIGH)
    lastEchoReceivedTime = micros();
}
