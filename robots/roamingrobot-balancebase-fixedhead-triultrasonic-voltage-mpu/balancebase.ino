///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

hw_timer_t *stepTimer = NULL;
void IRAM_ATTR onStepTimer();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupBase(){
  Serial.println("[SETUP] Gyro calibration starting...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // ESP32 hardware timer at 1MHz (20 ticks = 20us interrupt interval),
  // matching the original ATmega Timer2 ISR. Uses ESP32 Arduino Core v3.x API.
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 20, true, 0);
  
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 

  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading at register 43
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 6);                                      //Request 6 bytes (Gyro X, Y, Z)
    gyro_pitch_calibration_value += (int16_t)(Wire.read()<<8|Wire.read());  //Gyro X = pitch
    Wire.read(); Wire.read();                                               //Skip Gyro Y
    gyro_yaw_calibration_value += (int16_t)(Wire.read()<<8|Wire.read());    //Gyro Z = yaw
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
  Serial.print("[SETUP] Calibration done. pitch_cal=");
  Serial.print(gyro_pitch_calibration_value);
  Serial.print(" yaw_cal=");
  Serial.println(gyro_yaw_calibration_value);

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loopBase(){
  /*
  if(Serial.available()){                                                   //If there is serial data available
    received_byte = Serial.read();                                          //Load the received serial data in the received_byte variable
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted
  */
  // Battery voltage monitoring. 85 is the voltage compensation for the diode.
  // ESP32: 12-bit ADC (0-4095), 3.3V reference. Voltage divider must be redesigned
  // for 3.3V max input. With 10k/3.3k divider: 12.5V→~3.1V, multiplier ≈ 0.325.
  // The variable battery_voltage holds ~1050 if the battery voltage is 10.5V.
  // ADJUST this multiplier to match your actual resistor values.
  battery_voltage = (analogRead(BATTERY_ADC_PIN) * 0.325) + 85;
  
  if(battery_voltage < 850 && battery_voltage > 700){                      //If batteryvoltage is below 10.5V and higher than 8.0V
    digitalWrite(LED_PIN, HIGH);
    low_bat = 1;                                                            //Set the low_bat variable to 1
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3D);                                                         //Start reading at register 3D (Accel Y)
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = (int16_t)(Wire.read()<<8|Wire.read());           //Cast to int16_t for correct sign on ESP32
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 6);                                        //Request 6 bytes from the gyro (X, Y, Z)
  gyro_pitch_data_raw = (int16_t)(Wire.read()<<8|Wire.read());              //Gyro X = pitch (rotation around wheel axle)
  Wire.read(); Wire.read();                                                 //Skip Gyro Y (unused in this orientation)
  gyro_yaw_data_raw = (int16_t)(Wire.read()<<8|Wire.read());               //Gyro Z = yaw (rotation around vertical axis)
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -max_target_speed/2)pid_setpoint -= 0.01;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < max_target_speed/2)pid_setpoint += 0.01;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed/3)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  onStepTimer (ESP32 hardware timer, 20us interval)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ESP32 timer ISR — IRAM_ATTR required so it runs from IRAM, not flash.
// Uses GPIO.out_w1ts/w1tc for GPIO 0-31 (left motor) and GPIO.out1_w1ts/w1tc
// for GPIO 32-39 (right motor) for fast direct register access within the 20us deadline.
void IRAM_ATTR onStepTimer(){
  // Left motor pulse calculations
  throttle_counter_left_motor++;
  if(throttle_counter_left_motor > throttle_left_motor_memory){
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor;
    if(throttle_left_motor_memory < 0){
      GPIO.out_w1tc = (1 << LEFT_DIR_PIN);                                   // DIR LOW = reverse
      throttle_left_motor_memory *= -1;
    }
    else GPIO.out_w1ts = (1 << LEFT_DIR_PIN);                                // DIR HIGH = forward
  }
  else if(throttle_counter_left_motor == 1) GPIO.out_w1ts = (1 << LEFT_STEP_PIN);  // Step pulse HIGH
  else if(throttle_counter_left_motor == 2) GPIO.out_w1tc = (1 << LEFT_STEP_PIN);  // Step pulse LOW

  // Right motor pulse calculations (DIR is inverted: motors face opposite directions)
  // GPIO 32-39 use GPIO.out1_w1ts/w1tc registers with bit offset (pin - 32)
  throttle_counter_right_motor++;
  if(throttle_counter_right_motor > throttle_right_motor_memory){
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor;
    if(throttle_right_motor_memory < 0){
      GPIO.out1_w1ts.val = (1 << (RIGHT_DIR_PIN - 32));                      // DIR HIGH = reverse (inverted)
      throttle_right_motor_memory *= -1;
    }
    else GPIO.out1_w1tc.val = (1 << (RIGHT_DIR_PIN - 32));                   // DIR LOW = forward (inverted)
  }
  else if(throttle_counter_right_motor == 1) GPIO.out1_w1ts.val = (1 << (RIGHT_STEP_PIN - 32));  // Step pulse HIGH
  else if(throttle_counter_right_motor == 2) GPIO.out1_w1tc.val = (1 << (RIGHT_STEP_PIN - 32));  // Step pulse LOW
  triggerSonarInterrupt();
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
