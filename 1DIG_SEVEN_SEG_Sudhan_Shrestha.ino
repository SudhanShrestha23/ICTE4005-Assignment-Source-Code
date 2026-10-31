/*
* Author: Sudhan Shresta
* 
* This file is for an Arduino conected to a one-digit seven segement display, a LED and a buzzer. 
* The arduino takes in readings from a parent arduino for gas levels in a chamber and the number of operational fans
* The buzzer will raise an alert when there is a high amount of gas present OR when all funs are not operational
* A push button is used to switch between displaying gas levels and fan count where the LED state indicates which mode the display is in
*/

#include <Wire.h>


// I2C CONFIGURATION
const byte DISPLAY_ADDRESS = 0x09;

//                Order: a  b  c  d  e  f  g  dp
const byte segPins[8] = {7, 6, 4, 3, 2, 8, 9, 5};

const bool COMMON_ANODE = true;

const int BUZZER_PIN = 11;
const int DISPLAY_LED_PIN = 12;
const int BUTTON_PIN = 13;



bool fanMode = false;


int currentButtonState = LOW;
int previousButtonState = LOW;

unsigned long lastPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;


// RECEIVED SENSOR DATA


volatile byte receivedFanMask = 0;
volatile byte receivedFanSpeed = 0;
volatile byte receivedFlags = 0;
volatile byte receivedGas = 0;
volatile bool newData = false;

byte fanMask = 0;
byte faultFlags = 0;
byte gasLevel = 0;
byte fanSpeed = 0;

int workingFans = 0;
bool blockageFault = false;


// BUZZER VARIABLES

bool alarmActive = false;
bool buzzerOn = false;

unsigned long lastBuzzerChange = 0;
// 500 ms
const unsigned long BUZZER_INTERVAL = 500;


// SEVEN SEGMENT NUMBERS


const byte number[10][8] = {
  {1,1,1,1,1,1,0,0},  // 0
  {0,1,1,0,0,0,0,0},  // 1
  {1,1,0,1,1,0,1,0},  // 2
  {1,1,1,1,0,0,1,0},  // 3
  {0,1,1,0,0,1,1,0},  // 4
  {1,0,1,1,0,1,1,0},  // 5
  {1,0,1,1,1,1,1,0},  // 6
  {1,1,1,0,0,0,0,0},  // 7
  {1,1,1,1,1,1,1,0},  // 8
  {1,1,1,1,0,1,1,0}   // 9
};

// DISPLAY NUMBER ON SEVEN SEGEMENT

void showNumber(int n) {

  for (int i = 0; i < 8; i++) {

    bool lit = (n >= 0 && n <= 9) ? number[n][i] : 0;

    digitalWrite(
      segPins[i],
      COMMON_ANODE ? !lit : lit
    );
  }
}


// RECEIVE I2C DATA

void receiveEvent(int count) {

  if (count != 4) {

    while (Wire.available()) {
      Wire.read();
    }

    return;
  }

  receivedFanMask = Wire.read();
  receivedFanSpeed = Wire.read();
  receivedFlags = Wire.read();
  receivedGas = Wire.read();

  newData = true;
}


// PROCESS RECEIVED DATA


void updateData() {

  if (!newData) {
    return;
  }

  noInterrupts();

  fanMask = receivedFanMask;
  fanSpeed = receivedFanSpeed;
  faultFlags = receivedFlags;
  gasLevel = receivedGas;

  newData = false;

  interrupts();

  // Count working fans from bits 0-2

  workingFans = 0;

  for (int i = 0; i < 3; i++) {

    if (fanMask & (1 << i)) {
      workingFans++;
    }
  }

  // Bit 0 indicates blockage

  blockageFault = (faultFlags & (1 << 0)) != 0;

  // Alarm if blockage OR all fans off

  alarmActive = blockageFault || (workingFans == 0);
}


// CHECK BUTTON


void checkButton() {

  currentButtonState = digitalRead(BUTTON_PIN);

  // Detect LOW to HIGH transition

  if (currentButtonState == HIGH && previousButtonState == LOW && millis() - lastPressTime > DEBOUNCE_DELAY) {

    lastPressTime = millis();

    fanMode = !fanMode;

    Serial.println( fanMode ? "Switched to FAN display" : "Switched to GAS display");
  }

  previousButtonState = currentButtonState;
}


// DISPLAY GAS LEVEL by dividing percentage by 10 to be shown in display

void displayGas() {

  digitalWrite(DISPLAY_LED_PIN, LOW);

  int displayValue = gasLevel / 10;

  if (displayValue > 9) {
    displayValue = 9;
  }

  showNumber(displayValue);
}


// DISPLAY WORKING FANS


void displayFans() {

  digitalWrite(DISPLAY_LED_PIN, HIGH);

  // Display number of working fans
  showNumber(workingFans);
}


// BUZZER CONTROL
void updateBuzzer() {

  if (!alarmActive) {

    noTone(BUZZER_PIN);

    buzzerOn = false;
    return;
  }

  // Beep for 500ms, off for 500ms

  if (millis() - lastBuzzerChange >= BUZZER_INTERVAL) {

    lastBuzzerChange = millis();

    buzzerOn = !buzzerOn;

    if (buzzerOn) {
      tone(BUZZER_PIN, 3000);
    }
    else {
      noTone(BUZZER_PIN);
    }
  }
}


// PRINTING TO SERIAL MONITOR


void printStatus() {

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 500) {
    return;
  }

  lastPrint = millis();

  Serial.print("Gas: ");
  Serial.print(gasLevel);

  Serial.print("% | Working Fans: ");
  Serial.print(workingFans);


  Serial.print(" | Fault Flags: ");
  Serial.print(faultFlags, BIN);

  Serial.print(" | Display Mode: ");
  Serial.print(fanMode ? "FANS" : "GAS");

  Serial.print(" | Alarm: ");
  Serial.println(alarmActive ? "ON" : "OFF");
}

// SETUP

void setup() {

  Serial.begin(9600);

  for (int i = 0; i < 8; i++) {
    pinMode(segPins[i], OUTPUT);
  }

  pinMode(BUTTON_PIN, INPUT);
  pinMode(DISPLAY_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(DISPLAY_LED_PIN, LOW);

  showNumber(0);

  // Arduino 3 is I2C slave
  Wire.begin(DISPLAY_ADDRESS);

  Wire.onReceive(receiveEvent);

  Serial.println("Display Arduino Started");
}


// MAIN LOOP

void loop() {

  updateData();

  checkButton();

  if (fanMode) {
    displayFans();
  }
  else {
    displayGas();
  }

  updateBuzzer();

  printStatus();
}