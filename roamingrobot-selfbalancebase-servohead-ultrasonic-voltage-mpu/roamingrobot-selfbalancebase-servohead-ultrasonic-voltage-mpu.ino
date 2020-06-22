#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

//functional Configuration
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 5;                                    //Turning speed (20)
float max_target_speed = 5;                                //Max target speed (100)
int acc_calibration_value = 525;                            //Enter the accelerometer calibration value
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

void setup() {
  Serial.begin(9600);
  setupBase();
}

void loop() {
  /*
   * Left B00000001
   * Right B00000010
   * Forward B00000100
   * backward B00001000
   * DontKnow B00001100
   */
  received_byte = 0x00;
  loopBase();
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}
