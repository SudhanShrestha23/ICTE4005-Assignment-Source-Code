#include <Wire.h>
#include <Servo.h>

// PIN CONFIGURATION

const byte FAN_BUTTONS[3] = {2, 3, 4};
const byte FAN_LEDS[3] = {5, 6, 7};
const byte SERVO_PIN = 9;

const byte I2C_ADDRESS = 0x08;

// SENSOR THRESHOLDS

const int BLOCKAGE_TRIGGER = 15;
const int BLOCKAGE_CLEAR = 20;

const int VIBRATION_TRIGGER = 20;
const int VIBRATION_CLEAR = 12;

const unsigned long SENSOR_TIMEOUT = 100000;

// FAN VARIABLES

bool manualFailed[3] = {false, false, false};
bool fanOn[3] = {false, false, false};

bool blockageFault = false;
bool vibrationFault = false;
bool sensorFault = true;

int gasLevel = 0;
int distance = -1;
int vibration = 0;

int workingFans = 0;
float rpm = 0;

// I2C COMMUNICATION

// Incoming 6-byte sensor packet
volatile byte receivedPacket[6];
volatile bool newPacket = false;
volatile unsigned long lastPacketTime = 0;

// Outgoing 4-byte status packet
volatile byte statusPacket[4] = {0, 0, 4, 0};

// SERVO VARIABLES

Servo fanServo;

int servoPosition = 0;
int servoDirection = 1;

unsigned long lastServoStep = 0;
unsigned long servoInterval = 0;

bool servoAttached = false;

// BUTTON DEBOUNCING
bool lastButtonReading[3] = {HIGH, HIGH, HIGH};
bool stableButtonState[3] = {HIGH, HIGH, HIGH};

unsigned long lastDebounceTime[3] = {0, 0, 0};

const unsigned long DEBOUNCE_DELAY = 40;

unsigned long lastPrintTime = 0;

// recieving sensor data
void receiveEvent(int count) {

if (count != 6) {
while (Wire.available()) {
Wire.read();
}
return;
}

for (int i = 0; i < 6; i++) {
receivedPacket[i] = Wire.read();
}

lastPacketTime = millis();
newPacket = true;
}

//sending sensor data
void requestEvent() {

Wire.write((const byte*)statusPacket, 4);
}

void setup() {

  Serial.begin(9600);
  
  for (int i = 0; i < 3; i++) {
    
    pinMode(FAN_BUTTONS[i], INPUT_PULLUP);
    
    pinMode(FAN_LEDS[i], OUTPUT);
    digitalWrite(FAN_LEDS[i], LOW);
  }
  
  Wire.begin(I2C_ADDRESS);
  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  Serial.println("Fan Controller Started");
}

void readButtons() {

  for (int i = 0; i < 3; i++) {
  
  bool reading = digitalRead(FAN_BUTTONS[i]);
  
  if (reading != lastButtonReading[i]) {
  lastDebounceTime[i] = millis();
  }
  
  if (millis() - lastDebounceTime[i] > DEBOUNCE_DELAY) {
  
  if (reading != stableButtonState[i]) {
  
  stableButtonState[i] = reading;
  
  if (reading == LOW) {
  
  manualFailed[i] = !manualFailed[i];
  
  Serial.print("Fan ");
  Serial.print(i + 1);
  
  if (manualFailed[i]) {
  Serial.println(" manually failed");
  } else {
  Serial.println(" manually restored");
  }
  }
  }
  }
  
  lastButtonReading[i] = reading;
  }
}

// PROCESS SENSOR DATA
void readSensorData() {
  
  byte packet[6];
  bool hasPacket = false;
  unsigned long packetTime;
  
  noInterrupts();
  
  if (newPacket) {
  
  for (int i = 0; i < 6; i++) {
  packet[i] = receivedPacket[i];
  }
  
  newPacket = false;
  hasPacket = true;
  }
  
  packetTime = lastPacketTime;
  
  interrupts();
  
  if (hasPacket) {
  
  gasLevel = constrain((int)packet[0], 0, 100);
  
  distance = (int16_t)(
  (uint16_t)packet[1] |
  ((uint16_t)packet[2] << 8)
  );
  
  vibration = (uint16_t)(
  (uint16_t)packet[3] |
  ((uint16_t)packet[4] << 8)
  );
  }
  
  bool timedOut =
  millis() - packetTime > SENSOR_TIMEOUT;
  
  bool invalidReading =
  distance == -1 || vibration == 65535; //I don't think distance can be invalid anymore since no reading is classified as no blockage
  
  sensorFault = timedOut || invalidReading;
  
  if (sensorFault) {
  blockageFault = false;
  vibrationFault = false;
  return;
  }
  
  // Blockage detection with hysteresis
  if (distance <= BLOCKAGE_TRIGGER) {
  blockageFault = true;
  }
  else if (distance >= BLOCKAGE_CLEAR) {
  blockageFault = false;
  }
  // Vibration detection with hysteresis
  if (vibration >= VIBRATION_TRIGGER) {
  vibrationFault = true;
  }
  else if (vibration <= VIBRATION_CLEAR) {
  vibrationFault = false;
  }
}

// UPDATES FAN STATES BASED ON FAULTS (EITHER BUTTONS OR SENSOR FAULTS)
void updateFans() {
  workingFans = 0;
  
  for (int i = 0; i < 3; i++) {
  
  fanOn[i] = !manualFailed[i];
  
  // Vibration fault shuts down Fan 1
  if (i == 0 && vibrationFault) {
  fanOn[i] = false;
  }
  
  // Blockage or sensor failure shuts all fans
  if (blockageFault || sensorFault) {
  fanOn[i] = false;
  }
  
  digitalWrite(
  FAN_LEDS[i],
  fanOn[i] ? HIGH : LOW
  );
  
  if (fanOn[i]) {
  workingFans++;
  }
  }
  }
  
  // based on gas level and number of working fans
  void calculateRPM() {
  
  if (workingFans == 0 || gasLevel == 0) {
  
  rpm = 0;
  servoInterval = 0;
  return;
  }
  
  // 3 fans = gas / 10
  // 2 fans = gas / 10 * 2
  // 1 fan = gas / 10 * 3
  
  rpm = (gasLevel / 5) * (4 - workingFans);
  
  // One revolution = 180 degrees
  //
  // Time per revolution (microseconds):
  // 60,000,000 / RPM
  //
  // Time per degree:
  // 60,000,000 / (RPM * 180)
  
  servoInterval =
  (unsigned long)(60000000.0 / (rpm * 180.0));

  if (servoInterval < 1000) {
    servoInterval = 1000;
  }
}


// MOVE SERVO
void updateServo() {

  if (rpm <= 0) {
  
    if (servoAttached) {
      fanServo.detach();
      servoAttached = false;
    }
  
    return;
  }
  
  if (!servoAttached) {
  
  fanServo.attach(SERVO_PIN);
  fanServo.write(servoPosition);
  
  servoAttached = true;
  lastServoStep = micros();
  
  return;
  }
  
  unsigned long currentTime = micros();
  
  if (currentTime - lastServoStep >= servoInterval) {
  
  lastServoStep += servoInterval;
  
  servoPosition += servoDirection;
  
  if (servoPosition >= 180) {
  
  servoPosition = 180;
  servoDirection = -1;
  }
  
  if (servoPosition <= 0) {
  
  servoPosition = 0;
  servoDirection = 1;
  }
  
  fanServo.write(servoPosition);
  }
}

// UPDATE I2C STATUS
void updateStatus() {
  
  byte fanMask = 0;
  byte flags = 0;
  
  for (int i = 0; i < 3; i++) {
  
    if (fanOn[i]) {
      fanMask |= (1 << i);
    }
    
    if (manualFailed[i]) {
      flags |= (1 << 3);
    }
  }
  
  if (blockageFault) {
    flags |= (1 << 0);
  }
  
  if (vibrationFault) {
    flags |= (1 << 1);
  }
  
  if (sensorFault) {
    flags |= (1 << 2);
  }
  
  // Percentage of maximum demo RPM (150)
  byte speedPercent =
  (byte)constrain((int)(rpm * 60 / 150.0), 0, 100);
  
  noInterrupts();
  
  statusPacket[0] = fanMask;
  statusPacket[1] = speedPercent;
  statusPacket[2] = flags;
  statusPacket[3] = (byte)gasLevel;
  
  interrupts();
}

// SERIAL MONITOR
void printStatus() {
  if (millis() - lastPrintTime < 500) {
    return;
  }
  lastPrintTime = millis();
  
  Serial.print("Gas: ");
  Serial.print(gasLevel);
  Serial.print("%");
  Serial.print(" | Working Fans: ");
  Serial.print(workingFans);
  Serial.print(" | RPM: ");
  Serial.print(rpm, 1);
  Serial.print(" | Position: ");
  Serial.print(servoPosition);
  Serial.print(" | Fan States: ");

for (int i = 0; i < 3; i++) {
  Serial.print(fanOn[i] ? "ON" : "OFF");
  if (i < 2) {
  Serial.print(", ");
  }
  }
  if (sensorFault) {
  Serial.print(" | SENSOR FAULT");
  }
  if (blockageFault) {
  Serial.print(" | BLOCKAGE");
  }
  if (vibrationFault) {
  Serial.print(" | VIBRATION");
  }
  Serial.println();
}

void loop() {

  readButtons();
  readSensorData();
  updateFans();
  calculateRPM();
  updateServo();
  updateStatus();
  printStatus();
}
