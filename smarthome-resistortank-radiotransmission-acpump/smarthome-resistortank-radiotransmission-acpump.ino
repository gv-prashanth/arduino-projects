/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with
   HT12E (Encoder)and HT12D (Decoder) ICs for water level Data communiction and finally
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank bottom water level sensor attached to pin 2 with 10k resistor to ground
   Overhed tank top water level sensor attached to pin 3 with 10k resistor to ground
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
   Valid transmition Indicates that the transmitter is alive! attached to pin A0 with 10k resistor to ground
*/
//Pin Configurations
const int overheadTransmissionBottomPin = 9; // Over head tank water Minimum level
const int overheadTransmissionTopPin = 10; // Over head tank water Maximum level
const int overheadTransmissionValidityPin = A0; // Valid transmition  pin
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpDangerIndicatorPin = 11; // Sump danger level led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 10000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long PROTECTION_FOR_DRY_RUN = 3600000; //in milliseconds

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime;
boolean cached_overheadBottomHasWater, cached_overheadTopHasWater;
boolean firstTimeStarting, motorInDanger, isMotorRunning;

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(9600);

  // initialize the sumpMotorTriggerPin pin, SumpDangerIndicatorPin as Output
  pinMode(sumpMotorTriggerPin, OUTPUT);
  pinMode(SumpDangerIndicatorPin, OUTPUT);

  // initialize the overheadTransmissionBottomPinPin, overheadTransmissionTopPinPin as an input
  pinMode(overheadTransmissionBottomPin, INPUT);
  pinMode(overheadTransmissionTopPin, INPUT);

  // initialize all the necessary readings
  unsigned long currentTime = millis();
  lastSuccesfulOverheadTransmissionTime = currentTime;
  lastSwitchOnTime = currentTime;
  lastSwitchOffTime = currentTime;
  cached_overheadBottomHasWater = true;
  cached_overheadTopHasWater = true;
  firstTimeStarting = true;
  motorInDanger = false;
  isMotorRunning = false;
}

void loop()
{
  // read values from all sensors
  loadDangerAttribute();
  loadAndCacheOverheadTransmissions();

  //Based on read values decide to switch on or off
  if (isMotorRunning && !isConnectionWithinTreshold()) {
    switchOffMotor();
    Serial.println("No strong signal from overhead. Shutting down!");
  } else if (isMotorRunning && motorInDanger) {
    switchOffMotor();
    Serial.println("Danger! Out of water in sump. Shutting down!");
  } else if (isMotorRunning && cached_overheadTopHasWater) {
    switchOffMotor();
    Serial.println("Overhead tank is filled. Shutting down!");
  } else if (!isMotorRunning && !motorInDanger && !cached_overheadBottomHasWater) {
    if(isRecentlySwitchedOff()){
      Serial.println("No water in overhead tank. But very recently switched Off. Lets wait for protection time and then check.");
    }else{
      switchOnMotor();
      Serial.println("No water in overhead tank. Also sump has water. Switching ON!");
    }
  } else {
    //Serial.println("Lets leave the motor in whatever state it is in.");
  }
}

void switchOffMotor() {
  lastSwitchOffTime = millis();
  digitalWrite(sumpMotorTriggerPin, LOW);
  isMotorRunning = false;
}

void switchOnMotor() {
  if (firstTimeStarting)
    firstTimeStarting = false;
  lastSwitchOnTime = millis();
  digitalWrite(sumpMotorTriggerPin, HIGH);
  isMotorRunning = true;
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  return currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME ;
}

void loadAndCacheOverheadTransmissions() {
  boolean isOverheadTransmissionValid = digitalRead(overheadTransmissionValidityPin);
  if (isOverheadTransmissionValid) {
    cached_overheadBottomHasWater = digitalRead(overheadTransmissionBottomPin);
    cached_overheadTopHasWater = digitalRead(overheadTransmissionTopPin);
    lastSuccesfulOverheadTransmissionTime = millis();
  } else {
    Serial.println("Didnt get any signal from overhead. Will check again in sometime. Untill then use the cached values.");
  }
}

boolean loadDangerAttribute() {
  if(!motorInDanger)
    motorInDanger = millis()-lastSwitchOnTime > PROTECTION_FOR_DRY_RUN;
  if (motorInDanger)
    digitalWrite(SumpDangerIndicatorPin, HIGH);
}

boolean isRecentlySwitchedOff(){
  return (!firstTimeStarting) && (millis() - lastSwitchOffTime < PROTECTION_BETWEEN_SWITCH_OFF_ON);
}
