#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

//Pin Configuration
const int TRIGGER_PIN = 7; // Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 6; // Arduino pin tied to echo pin on ping sensor.

//functional Configuration
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 5;                                    //Turning speed (20)
float max_target_speed = 5;                                //Max target speed (100)
int acc_calibration_value = 675;                            //Enter the accelerometer calibration value
int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)

const int timeToStickRightLeftDecission = 5000;//milli seconds

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

byte commandToRun;
unsigned long commandEndTime, botStartTime, lastSonarSentTime, lastEchoReceivedTime;
unsigned long lastRightLeftDecidedTime = 0;
boolean isRightDecidedCached = false;

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
  setupBase();
  botStartTime = millis();
  lastSonarSentTime = micros();
  lastEchoReceivedTime = micros()+10000;
}

void loop() {
  long obstacleDist = (lastEchoReceivedTime - lastSonarSentTime) / 58.2;
  //Serial.println(obstacleDist);
  if (micros() - lastSonarSentTime > 250000) {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    lastSonarSentTime = micros();
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(17);
    digitalWrite(TRIGGER_PIN, LOW);
  }

  if (millis() - botStartTime > 10000) {
    if (obstacleDist < 80)
      if(isRightDecided())
        runCommandFor(B00000010, 8000);
      else
        runCommandFor(B00000001, 8000);
    else
      runCommandFor(B00000100, 1000);
  }
  loadCommand();
  loopBase();
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while (loop_timer > micros());
  loop_timer += 4000;
}

void runCommandFor(byte command, unsigned long timeToRun) {
  /*
    Left B00000001
    Right B00000010
    Forward B00000100
    backward B00001000
    DontKnow B00001100
    DontKnow 0x00
  */
  commandEndTime = millis() + timeToRun;
  commandToRun = command;
}

void loadCommand() {
  if (commandEndTime > millis()) {
    received_byte = commandToRun;
  } else {
    received_byte = 0x00;
  }
}

boolean isRightDecided() {
  if (millis() - lastRightLeftDecidedTime > timeToStickRightLeftDecission) {
    isRightDecidedCached = (random(0, 2) < 1);
    lastRightLeftDecidedTime = millis();
  }
  return isRightDecidedCached;
}

//Within interrupt. Should run fast.
void triggerSonarInterrupt() {
  if(digitalRead(ECHO_PIN) == HIGH)
    lastEchoReceivedTime = micros();
}
