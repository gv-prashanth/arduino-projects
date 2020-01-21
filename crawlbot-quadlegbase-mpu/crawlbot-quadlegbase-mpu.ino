#include <Servo.h>
#include <math.h>
#include <PID_v1.h> //You can download the driver from https://github.com/br3ttb/Arduino-PID-Library/

//setups
const int legPins[4][3] = { // [FrontLeft, FrontRight, BackLeft, BackRight][Coxa, Femur, Tibia]
  {11, 12, 13},
  {8, 9, 10},
  {2, 3, 4},
  {5, 6, 7}
};
const int speakerPin = 48;
const double armLength = 4;//cm
double Kp = 0.75, Ki = 0.0, Kd = 0.0; //Specify the links and initial tuning parameters

//configs
const double defaultHeight = 7.00;//cm. Should be less than 2*armLength. TODO: later this will be incorprated by vision sensor
const double maxSwingAngle = 30;//degrees. Should be less than arccos(h/2r). TODO: later this will be incorporated by height
const double timerStepper = PI / 32; //TODO: later this will be incorported as speed. Better to be multiple of 8
const double sideSwingAngle = 6;//degrees. Why is this even there? It should auto decide based on its balace MPU

//dont touch below variables
Servo legServos[4][3];// [FrontLeft, FrontRight, BackLeft, BackRight][Coxa, Femur, Tibia]
double cftAnglesMatrix[4][3], cftNumbersMatrix[4][3];// [FrontLeft, FrontRight, BackLeft, BackRight][Coxa, Femur, Tibia]
double heightMatrix[4];// [FrontLeft, FrontRight, BackLeft, BackRight]
double timerNumber = 0;
double Setpoint, Input, Output;//Define Variables we'll be connecting to
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin (115200);
  attachServosToPins();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-0.5, 0.5);

  Serial.println("Waiting for you to test...");
  bodyTestSequence();
  tone(speakerPin, 1000, 2000);
  delay(2000);
  Serial.println("Hope you have tested my legs are at 90 degrees...");

  bodyStartSequence();
  Serial.println("Waiting for body to stabilize...");
  tone(speakerPin, 2000, 2000);
  delay(2000);
  Serial.println("Body stabilized. Ready to rock...");

  setupMPU();
  stabilizeMPU();

}



void loop() {
  //incrementTimerNumber();
  populateYPR();
  populateHeightArrayBaseOnMPUReadings();
  populateCFTNumbersMatrixBasedOnHeightArrayAndTimerNumber(); //printCFTNumbersMatrix();
  populateCFTAnglesMatrixBasedOnCFTNumbersMatrix(); //printCFTAnglesMatrix();
  adjustLegsBasedOnCFTAnglesMatrix();
}

void populateCFTNumbersMatrixBasedOnHeightArrayAndTimerNumber() {
  //saving in temp for later identifying lifgint leg from behind scenario
  double tempFrontLeftFemurHistory = cftNumbersMatrix[0][1];
  double tempFrontRightFemurHistory = cftNumbersMatrix[1][1];
  double tempBackLeftFemurHistory = cftNumbersMatrix[2][1];
  double tempBackRightFemurHistory = cftNumbersMatrix[3][1];

  //Logic to assign cftnumbers based on time based femur number and calculated tibia number
  //See the old commented method below for a easier understanding
  for (int i = 0; i < 4; i++) {
    cftNumbersMatrix[i][0] = 0;
    cftNumbersMatrix[i][1] = getFemurNumberFromTimerNumber(timerNumber + getOffsetByIndex(i));
    cftNumbersMatrix[i][2] = getTibiaFromFemurAndHeightAndLegIndex(i, getFemurNumberFromTimerNumber(timerNumber + getOffsetByIndex(i)), heightMatrix[i]);
  }

  //lifting legs from behind and placing forward logic is below
  //Also Creep gait logic of bending left right is also below
  if (tempFrontLeftFemurHistory < 0 && cftNumbersMatrix[0][1] > 0) {
    //Frontleft needs to be lift up and placed forward
    cftNumbersMatrix[0][2] = 10 + getTibiaFromFemurAndHeightAndLegIndex(0, tempFrontLeftFemurHistory, heightMatrix[0]);
    cftNumbersMatrix[1][0] = sideSwingAngle;
    cftNumbersMatrix[2][0] = sideSwingAngle;
    cftNumbersMatrix[3][0] = sideSwingAngle;
  }
  if (tempFrontRightFemurHistory < 0 && cftNumbersMatrix[1][1] > 0) {
    //Frontright needs to be lift up and placed forward
    cftNumbersMatrix[1][2] = 10 + getTibiaFromFemurAndHeightAndLegIndex(1, tempFrontRightFemurHistory, heightMatrix[1]);
    cftNumbersMatrix[0][0] = -1 * sideSwingAngle;
    cftNumbersMatrix[2][0] = -1 * sideSwingAngle;
    cftNumbersMatrix[3][0] = -1 * sideSwingAngle;
  }
  if (tempBackLeftFemurHistory < 0 && cftNumbersMatrix[2][1] > 0) {
    //Backleft needs to be lift up and placed forward
    cftNumbersMatrix[2][2] = -10 + getTibiaFromFemurAndHeightAndLegIndex(2, cftNumbersMatrix[2][1], heightMatrix[2]);
    cftNumbersMatrix[0][0] = sideSwingAngle;
    cftNumbersMatrix[1][0] = sideSwingAngle;
    cftNumbersMatrix[3][0] = sideSwingAngle;
  }
  if (tempBackRightFemurHistory < 0 && cftNumbersMatrix[3][1] > 0) {
    //Backright needs to be lift up and placed forward
    cftNumbersMatrix[3][2] = -10 + getTibiaFromFemurAndHeightAndLegIndex(3, cftNumbersMatrix[3][1], heightMatrix[3]);
    cftNumbersMatrix[0][0] = -1 * sideSwingAngle;
    cftNumbersMatrix[1][0] = -1 * sideSwingAngle;
    cftNumbersMatrix[2][0] = -1 * sideSwingAngle;
  }
}

/*
  void populateCFTNumbersMatrixBasedOnHeightArrayAndTimerNumber(){
  double frontLeftFemurNumber = getFemurNumberFromTimerNumber(timerNumber);
  double frontRightFemurNumber = getFemurNumberFromTimerNumber(timerNumber-PI/2);
  double backLeftFemurNumber = getFemurNumberFromTimerNumber(timerNumber-3*PI/4);;
  double backRightFemurNumber = getFemurNumberFromTimerNumber(timerNumber-PI/4);
  cftNumbersMatrix[0][0] = 0;
  cftNumbersMatrix[0][1] = frontLeftFemurNumber;
  cftNumbersMatrix[0][2] = getTibiaFromFemurAndHeight_Front(frontLeftFemurNumber, heightMatrix[0]);
  cftNumbersMatrix[1][0] = 0;
  cftNumbersMatrix[1][1] = frontRightFemurNumber;
  cftNumbersMatrix[1][2] = getTibiaFromFemurAndHeight_Front(frontRightFemurNumber, heightMatrix[1]);
  cftNumbersMatrix[2][0] = 0;
  cftNumbersMatrix[2][1] = backLeftFemurNumber;
  cftNumbersMatrix[2][2] = getTibiaFromFemurAndHeight_Back(backLeftFemurNumber, heightMatrix[2]);
  cftNumbersMatrix[3][0] = 0;
  cftNumbersMatrix[3][1] = backRightFemurNumber;
  cftNumbersMatrix[3][2] = getTibiaFromFemurAndHeight_Back(backRightFemurNumber, heightMatrix[3]);
  }
*/

double getFemurNumberFromTimerNumber(double inputTimerNumber) {
  if (inputTimerNumber < 0) {
    return maxSwingAngle * cos(PI + inputTimerNumber);
  }
  if (inputTimerNumber > PI) {
    return maxSwingAngle * cos(inputTimerNumber - PI);
  }
  //TODO: I think below two if conditions are hacks only and need to be rethough and removed later
  if (inputTimerNumber == 0) {
    return maxSwingAngle * cos(timerStepper);
  }
  if (inputTimerNumber == PI) {
    return maxSwingAngle * cos(timerStepper);
  }
  return maxSwingAngle * cos(inputTimerNumber);
}

void incrementTimerNumber() {
  if (timerNumber + timerStepper < PI - timerStepper) {
    timerNumber = timerNumber + timerStepper;
  } else {
    timerNumber = timerStepper;
  }
}

double getTibiaFromFemurAndHeightAndLegIndex(int legIndex, double femurNumber, double height) {
  if (legIndex == 0 || legIndex == 1) {
    return getTibiaFromFemurAndHeight_Front(femurNumber, height);
  } else {
    return getTibiaFromFemurAndHeight_Back(femurNumber, height);
  }
}

double getTibiaFromFemurAndHeight_Front(double femurNumber, double height) {
  return (acos((height / armLength) - cos(femurNumber * (PI / 180))) * (180 / PI)) - (femurNumber);
}

double getTibiaFromFemurAndHeight_Back(double femurNumber, double height) {
  return -1 * ((acos((height / armLength) - cos(femurNumber * (PI / 180))) * (180 / PI)) + (femurNumber));
}

void attachServosToPins() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      legServos[i][j].attach(legPins[i][j]);
    }
  }
}

void bodyTestSequence() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      legServos[i][j].write(90);
    }
  }
  legServos[0][2].write(180);
  legServos[1][2].write(180);
  legServos[2][2].write(0);
  legServos[3][2].write(0);
}

void populateHeightArrayBaseOnMPUReadings() {
  //TODO: need to remove below code and write actual code
  heightMatrix[0] = defaultHeight;
  heightMatrix[1] = defaultHeight;
  heightMatrix[2] = defaultHeight;
  heightMatrix[3] = defaultHeight;

  Setpoint = 0;
  Input = ypr[2] * 180 / M_PI;
  myPID.Compute();
  //If powerDiff is positive i need to steer left
  //If powerDiff is negative i need to steer right
  float heightDiff = Output;
  Serial.println(heightDiff);

    heightMatrix[0] = defaultHeight+heightDiff;
    heightMatrix[1] = defaultHeight-heightDiff;
    heightMatrix[2] = defaultHeight+heightDiff;
    heightMatrix[3] = defaultHeight-heightDiff;

    Serial.println("left: "+String(heightMatrix[0])+" "+String(heightMatrix[2]));
    Serial.println("right: "+String(heightMatrix[1])+" "+String(heightMatrix[3]));
  /*
    //Considering pitch
    if(ypr[1]>3){
    //Front is high
    Serial.println("Front is high");
    tone(speakerPin, 5000, 50);
    }else if(ypr[1]<-3){
    //Back is high
    Serial.println("Back is high");
    tone(speakerPin, 5000, 50);
    }else{

    }

    //Considering Roll
    if(ypr[2]>3){
    //left is high
    Serial.println("left is high");
    tone(speakerPin, 500, 50);
    }else if(ypr[2]<-3){
    //right is high
    Serial.println("right is high");
    tone(speakerPin, 500, 50);
    }else{

    }
  */
}

void populateCFTAnglesMatrixBasedOnCFTNumbersMatrix() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (j == 2 && (i == 0 || i == 1)) {
        cftAnglesMatrix[i][j] = 180 - cftNumbersMatrix[i][j];
      } else if (j == 2 && (i == 2 || i == 3)) {
        cftAnglesMatrix[i][j] = -1 * cftNumbersMatrix[i][j];
      } else {
        cftAnglesMatrix[i][j] = 90 + cftNumbersMatrix[i][j];
      }
    }
  }
}

/*
  void populateCFTAnglesMatrixBasedOnCFTNumbersMatrix(){
  cftAnglesMatrix[0][0] = 90+cftNumbersMatrix[0][0];
  cftAnglesMatrix[0][1] = 90+cftNumbersMatrix[0][1];
  cftAnglesMatrix[0][2] = 180-cftNumbersMatrix[0][2];
  cftAnglesMatrix[1][0] = 90+cftNumbersMatrix[1][0];
  cftAnglesMatrix[1][1] = 90+cftNumbersMatrix[1][1];
  cftAnglesMatrix[1][2] = 180-cftNumbersMatrix[1][2];
  cftAnglesMatrix[2][0] = 90+cftNumbersMatrix[2][0];
  cftAnglesMatrix[2][1] = 90+cftNumbersMatrix[2][1];
  cftAnglesMatrix[2][2] = -1*cftNumbersMatrix[2][2];
  cftAnglesMatrix[3][0] = 90+cftNumbersMatrix[3][0];
  cftAnglesMatrix[3][1] = 90+cftNumbersMatrix[3][1];
  cftAnglesMatrix[3][2] = -1*cftNumbersMatrix[3][2];
  }
*/

void adjustLegsBasedOnCFTAnglesMatrix() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      legServos[i][j].write(cftAnglesMatrix[i][j]);
    }
  }
  delay(50);
}

void bodyStartSequence() {
  populateCFTNumbersMatrixWithStartPosture();
  printCFTNumbersMatrix();
  populateCFTAnglesMatrixBasedOnCFTNumbersMatrix();
  printCFTAnglesMatrix();
  adjustLegsBasedOnCFTAnglesMatrix();
}

void populateCFTNumbersMatrixWithStartPosture() {
  //quite simmilar to populateCFTNumbersMatrixBasedOnHeightArrayAndTimerNumber
  for (int i = 0; i < 4; i++) {
    heightMatrix[i] = defaultHeight;
  }
  for (int i = 0; i < 4; i++) {
    cftNumbersMatrix[i][0] = 0;
    cftNumbersMatrix[i][1] = getFemurNumberFromTimerNumber(0 + getOffsetByIndex(i));
    cftNumbersMatrix[i][2] = getTibiaFromFemurAndHeightAndLegIndex(i, getFemurNumberFromTimerNumber(0 + getOffsetByIndex(i)), heightMatrix[i]);
  }

}

double getOffsetByIndex(int index) {
  double offset = 0; //FL
  if (index == 1) {
    offset = 2 * PI / 4; //FR
  } else if (index == 2) {
    offset = 1 * PI / 4; //BL
  } else if (index == 3) {
    offset = 3 * PI / 4; //BR
  }
  return offset;
}

void printCFTNumbersMatrix() {
  Serial.println("printing CFT Numbers");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(cftNumbersMatrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void printCFTAnglesMatrix() {
  Serial.println("printing CFT Angles");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(cftAnglesMatrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
