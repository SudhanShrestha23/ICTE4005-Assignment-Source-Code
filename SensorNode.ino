#include <Wire.h>

// I2C addresses
const byte FAN_CONTROLLER = 0x08;// Address of the fan-controller Arduino
const byte DISPLAY_NODE = 0x09;//Address of the display Arduino
const byte MPU = 0x68; //Address of the MPU6050 sensor

// Sensor pins
const byte POT = A0;//Reads the potentiometer setting
const byte BTN = 4; //Reads the button
const byte TRIG = 9; //Sends the ultrasonic trigger pulse
const byte ECHO = 10;//Receives the ultrasonic echo


//If it is true, the code attempts to forward fan status to the display Arduino.
const bool DISPLAY_CONNECTED = true;

//unsigned long stores a large non-negative integer.This sets the intended interval between sensor-reading cycles to approximately 0.2 seconds, or five cycles per second.
const unsigned long UPDATE_INTERVAL = 200;

unsigned long lastUpdate = 0; //Stores the time when the previous update began.
byte sequenceNumber = 0; //Stores a number that increments each time sensor data is sent. This lets the receiving Arduino identify the order of packets and potentially detect missing or repeated packets.

//A struct groups several related variables into one data structure., Instead of keeping four separate variables for the fan status, the code groups them together.
struct FanStatus {
byte fanMask;//Bit pattern representing which fans are running
byte speed; //Fan speed, apparently represented as a percentage
byte flags;//Bit pattern representing fault conditions
byte gas;//Gas level reported by the fan controller
};
//Serial.begin(9600) starts serial communication at 9600 bits per second.
void setup() {
Serial.begin(9600);

pinMode(BTN, INPUT_PULLUP);//Sets pin 4 as an input with an internal pull-up resistor.
pinMode(TRIG, OUTPUT); //TRIG is an output because the Arduino sends a pulse to the ultrasonic sensor.
pinMode(ECHO, INPUT); //ECHO is an input because the Arduino measures the returning pulse.

digitalWrite(TRIG, LOW);//Sets the trigger pin LOW initially so that the sensor starts in a known state.

// This Arduino becomes the I2C master. Clock is set to 100 kHz (standard speed).
Wire.begin(); //Because no address is supplied, this Arduino becomes the I²C master. It initiates communication with the other devices.
Wire.setClock(100000); //Sets the I²C clock speed to 100,000 Hz, or 100 kHz.

// Calls the function that wakes up the MPU6050 accelerometer and gyroscope.
mpuStart();

Serial.println("Sensor Arduino Started");
}

//millis() returns the number of milliseconds since the Arduino started, lastUpdate stores the time of the previous update, millis() - lastUpdate calculates how much time has passed,UPDATE_INTERVAL is 200 milliseconds. if checks whether 200 milliseconds have passed
void loop() {
if (millis() - lastUpdate < UPDATE_INTERVAL) {
return;
}

lastUpdate = millis(); //Records the current time so the next update interval can be measured from this point.

// This line reads the potentiometer and converts its reading into a value from 0 to 100.. analogRead(POT), Reads the voltage on analogue pin A0. On a typical 10-bit Arduino analogue input, the result ranges from 0 to 1023.
byte gas = map(analogRead(POT), 0, 1023, 0, 100);//map(value, fromLow, fromHigh, toLow, toHigh), Converts a value from one range into another.

if (digitalRead(BTN) == LOW) { //digitalRead(BTN) reads the button's state., == LOW checks whether the button is pressed, If the button is pressed, gas = 0 sets the gas value to zero.
gas = 0;
}
int cm = readCm();//Calls readCm(), which measures the distance using the ultrasonic sensor.
unsigned int vibration = readVibe(); //Calls readVibe(), which measures changes in the MPU6050's accelerometer readings, If the sensor reading fails, readVibe() returns 65535 as an error indicator. The receiving controller needs to recognise this special value.

// Calls sendSensorData() and passes the three readings to it.The function returns either:true: the I²C transmission completed without a reported transmission erro, false: an error occurred during the transmission.
bool sent = sendSensorData(gas, cm, vibration);

// anStatus status; creates a variable that can store the four status bytes. gotStatus = false; initially assumes that no valid status has been received.
FanStatus status;
bool gotStatus = false;

//The code requests the fan status only if the sensor data was sent successfully.getFanStatus(status) attempts to receive four bytes and store them in the structure.
if (sent) { //The returned result is assigned to gotStatus. //This avoids requesting the fan status when the previous transmission failed.
gotStatus = getFanStatus(status);
}

// Both conditions must be true:The fan status was received successfully.The display Arduino is enabled in the code.If both conditions are true, the program forwards the four status bytes to the display Arduino.
if (gotStatus && DISPLAY_CONNECTED) {
sendDisplayData(status);
}

// 7. Print results on Serial Monitor
Serial.print("Gas: ");
Serial.print(gas);

Serial.print("% | Distance: ");
Serial.print(cm);

Serial.print(" cm | Vibration: ");
Serial.print(vibration);

if (gotStatus) { //Checks whether the program successfully received the fan status.//If it did, the program prints the status values.
Serial.print(" | Fan Mask: "); //Prints the fan mask in binary.//BIN tells Arduino to display the number in base 2 rather than decimal.
Serial.print(status.fanMask, BIN);

Serial.print(" | Fan Speed: ");
Serial.print(status.speed); //Prints the fan speed value.

Serial.print("% | Fault Flags: ");
Serial.print(status.flags, BIN); //Prints the speed unit and the fault flags in binary.
} else {
Serial.print(" | Controller communication error"); //If gotStatus is false, the program prints a communication error message instead of the fan status.
}

Serial.println(); //Serial.println() finishes the output line and moves to the next line.
}



bool sendSensorData(byte gas, int cm,
unsigned int vibration) { //This function sends six bytes to the fan-controller Arduino.

// bool means the function returns true or false.
//sendSensorData is the function name.
// byte gas receives the gas setting.
//int cm receives the distance.
// unsigned int vibration receives the vibration reading.

unsigned int distanceBits = (unsigned int)cm; //Converts cm to an unsigned integer and stores it in distanceBits.This prepares the distance value for transmission as two bytes.

Wire.beginTransmission(FAN_CONTROLLER); //tarts an I²C transmission to the device at address 0x08.

Wire.write(gas); //Sends the gas setting as one byte.

Wire.write(lowByte(distanceBits)); //Sends the distance as two separate bytes.A byte contains eight bits. A 16-bit value can be split into: Low byte: the lower eight bits.High byte: the upper eight bits.
Wire.write(highByte(distanceBits));

Wire.write(lowByte(vibration)); //Does the same thing for the vibration reading.
Wire.write(highByte(vibration));

Wire.write(sequenceNumber++); //Sends the current sequence number as the sixth byte.

return Wire.endTransmission() == 0; //Wire.endTransmission() finishes the transmission.A return value of 0 means the transmission succeeded according to the I²C library.
}

// ---------------------------------------
// REQUEST FAN STATUS
// ---------------------------------------

bool getFanStatus(FanStatus &status) { //This function requests the fan status and returns whether the expected data was received.

byte count =
Wire.requestFrom(FAN_CONTROLLER, (byte)4); //Requests four bytes from the fan controller at address 0x08.

if (count != 4) { //If the number of bytes received is not four, the function treats the response as invalid.
while (Wire.available()) { //Wire.available() checks whether unread bytes are available.while repeats the instructions as long as unread bytes remain.
Wire.read(); //Wire.read() reads and discards one byte.
}
return false; //Reports that the function failed to receive the expected four bytes.
}
//If four bytes were received, each byte is read and stored in the matching structure member.Fan mask.Fan speed.Fault flags.Gas level.
status.fanMask = Wire.read();
status.speed = Wire.read();
status.flags = Wire.read();
status.gas = Wire.read();

return true;//Reports that the expected number of bytes was received and stored.
}

// ---------------------------------------
// SEND STATUS TO DISPLAY ARDUINO
// ---------------------------------------

void sendDisplayData(const FanStatus &status) { //void means the function does not return a value.//const FanStatus &status passes the existing structure by reference but prevents the function from modifying it.

Wire.beginTransmission(DISPLAY_NODE); //Starts an I²C transmission to the display Arduino at address 0x09.
//Sends the four status bytes to the display Arduino in the expected order.
Wire.write(status.fanMask);
Wire.write(status.speed);
Wire.write(status.flags);
Wire.write(status.gas);

Wire.endTransmission();
}

// ---------------------------------------
// INITIALISE MPU6050
// ---------------------------------------

void mpuStart() {

Wire.beginTransmission(MPU);////Starts an I²C transmission to the MPU6050 at address 0x68.

Wire.write(0x6B); //Selects register 0x6B, which is the MPU6050's power-management register.
Wire.write(0x00); //Writes zero to that registerThe zero clears the sleep bit, waking up the MPU6050 so it can operate.

byte error = Wire.endTransmission(); //Finishes the I²C transmission and stores the result in error. A result of zero indicates that the transmission succeeded.

if (error != 0) {//If the transmission fails, the program prints an error message.
Serial.println("MPU6050 not detected!");
}
}

// ---------------------------------------
// READ ACCELEROMETER
// ---------------------------------------

bool readAcceleration(long &ax, long &ay, long &az) { //The function returns true if it reads the expected data successfully and false if communication fails.
//readAcceleration: the function name.long &ax: stores the X-axis reading.long &ay: stores the Y-axis reading.long &az: stores the Z-axis reading.
Wire.beginTransmission(MPU); //Starts an I²C transmission to the MPU6050, which has the address 0x68.
Wire.write(0x3B);//Tells the MPU6050 which register you want to read.

if (Wire.endTransmission(false) != 0) { //Wire.endTransmission(false) finishes sending the register address without sending a STOP condition.//
return false;//This allows the Arduino to perform a repeated-start operation when it requests the sensor data.
}

if (Wire.requestFrom(MPU, (byte)6) != 6) { //Requests six bytes from the MPU6050.//Why six bytes? Each acceleration axis requires two bytes.
while (Wire.available()) { //Wire.available() checks whether unread data is available.
Wire.read(); //Wire.read() reads one byte.
}
return false;
}

ax = (int16_t)((Wire.read() << 8) | Wire.read());//This reads two bytes and combines them into one 16-bit signed number.
ay = (int16_t)((Wire.read() << 8) | Wire.read());
az = (int16_t)((Wire.read() << 8) | Wire.read()); //After these lines, ax, ay and az contain the three raw acceleration readings.

return true;
}

// ---------------------------------------
// MEASURE VIBRATION
// ---------------------------------------
//step 1:The MPU6050 measures acceleration along three axes:
//step 2: Your code takes an initial reading, then takes 8 more readings, with an 8 ms delay between them.
//step 3: For each new reading, the code calculates the absolute difference between the current and previous readings for all three axes:
//step 4: vibration= total change/number of samples x 150

//My Arduino uses the MPU6050 accelerometer to measure movement along the X, Y and Z axes.
//It takes several readings and calculates the difference between consecutive readings.
//It adds up these differences and divides the total by a scaling factor to produce a vibration score. If the sensor moves or vibrates more, the score generally increases.”

unsigned int readVibe() {////Defines a function that returns an unsigned integer containing the vibration score.

long prevX, prevY, prevZ; //Creates three variables to store the previous X, Y and Z readings

if (!readAcceleration(prevX, prevY, prevZ)) { //This calls the readAcceleration() function explained above.
return 65535; //The value 65535 is reserved as a sensor-failure indicator.
}

long totalChange = 0; //Creates a variable that will accumulate all the changes in acceleration.

const int samples = 8; //Sets the number of additional readings to eight. The function takes one initial reading and then eight additional readings, giving nine readings in total.

for (int i = 0; i < samples; i++) { //Therefore, the loop executes eight times.

delay(8); //Waits for 8 milliseconds before taking the next measurement.

long x, y, z; //Creates variables for the new X, Y and Z readings.

if (!readAcceleration(x, y, z)) { //Attempts to read the new acceleration values.
return 65535; //If the sensor fails to provide the readings, the function returns 65535 immediately.
}

totalChange += labs(x - prevX); ///This line calculates the absolute change in the X-axis reading and adds it to totalChange,x - prevX: finds the difference between the new and previous readings.labs(): converts the difference into a non-negative absolute value.
totalChange += labs(y - prevY);//+=: adds that result to the existing total.
totalChange += labs(z - prevZ);

prevX = x; //Copies the current readings into the previous-reading variables.
prevY = y;
prevZ = z;
}

long result = totalChange / (samples * 150L); //This divides the accumulated change by the number of samples multiplied by 150.Step 7: Calculate the vibration score

if (result > 65534L) { //Checks whether the result exceeds 65534.If it does, the program sets it to 65534.This leaves 65535 available as the special sensor-failure value.
result = 65534L;
}

return (unsigned int)result;
}

//1. Read the initial acceleration Store the first X, Y and Z values.2. Take eight more readings.Wait 8 ms between readings. 3. Calculate the changes. Find absolute changes across all three axes.4. Calculate the score.Divide the total change by 1200 and return it.
// ---------------------------------------
// MEASURE ULTRASONIC DISTANCE
// ---------------------------------------
//step 1: Send an ultrasonic pulse
//2. Measure the time
//Calculate the distance duration/58

int readCm() { //This function measures the distance between the sensor and an object.

digitalWrite(TRIG, LOW); //Sets the ultrasonic sensor's trigger pin to LOW before starting a new measurement.
delayMicroseconds(2); //Waits two microseconds to ensure the signal is stable.

digitalWrite(TRIG, HIGH); //Step 2: Send a trigger pulse
delayMicroseconds(10); //Keeps the trigger pin HIGH for 10 microseconds

digitalWrite(TRIG, LOW);

unsigned long duration =
pulseIn(ECHO, HIGH, 30000UL); //Step 3: Measure the echo unsigned long: stores a non-negative time valuepulseIn(): measures the duration of a pulse.ECHO: the pin receiving the echo signal.HIGH: measures the time the pin remains HIGH30000UL: sets a timeout of 30,000 microseconds, equivalent to 30 milliseconds.

if (duration == 0) { //If pulseIn() returns zero, no valid echo pulse was measured before the timeout.The function returns 400 cm as a fallback value.
return 400;
}

return (int)(duration / 58); //Step 5: Convert the duration to centimetres
}

//My Arduino is the sensor Arduino, and its main job is to collect sensor data and send it to the fan controller.

//It uses a potentiometer to set the gas level, a button to reset that value to zero, an ultrasonic sensor to measure the distance to an object, and an MPU6050 accelerometer to measure vibration.

//Every 200 milliseconds, my Arduino reads these sensors and sends the data to the fan-controller Arduino using I2C communication. The data includes the gas setting, distance, vibration level and a sequence number to identify the order of the packets.

//After sending the data, my Arduino requests the fan controller's status, including which fans are running, their speed and any fault flags. It then forwards this information to the third Arduino, which is responsible for the display.

//Finally, my Arduino prints the sensor readings and fan status in the Serial Monitor, so we can see what's happening in the system.


//Why are you using I²C communication?
//I²C allows my Arduino to communicate with the fan controller and display Arduino using the SDA and SCL lines. Each device has an address so the correct device can be selected.

//Why does your code update every 200 milliseconds?
//It controls how frequently the Arduino starts another sensor-reading cycle. This gives an intended update interval of about five cycles per second, although the actual cycle time can be longer because of sensor readings and communication.

//How do you measure vibration?
//The MPU6050 measures acceleration along the X, Y and Z axes. My code compares consecutive readings, adds their absolute changes and divides by a scaling factor to produce a relative vibration score.

//What happens if a sensor fails?
//If the MPU6050 reading fails, my vibration function returns 65535 as an error indicator. If the ultrasonic sensor receives no echo before its timeout, the distance function returns 400 cm as a placeholder.

//Why does your Arduino receive data from the fan controller?
//It needs the controller's current fan status so it can print the information in the Serial Monitor and forward the status to the display Arduino.

//Does your Arduino control the fans directly?
//No. My Arduino collects sensor data and communicates with the fan controller. The fan controller is responsible for deciding how to operate the fans.

//how gas level is read.//The potentiometer sends an analogue voltage to pin A0. The Arduino reads this voltage and converts it into a number between 0 and 1023.//The map() function converts the reading from the range 0–1023 into the range 0–100.

//Q13. Why did you use I²C instead of connecting everything directly?
//I²C allows the Arduinos to exchange information using a small number of communication wires. Devices can also be identified using their addresses.

//Q14. What are the addresses 0x08, 0x09 and 0x68?
//They identify different devices on the I²C bus. In my code, 0x08 is the fan controller, 0x09 is the display Arduino, and 0x68 is the MPU6050 accelerometer.

//Q15. What data does your Arduino send to the fan controller?
//It sends six bytes: the gas level, distance split into two bytes, vibration split into two bytes, and a sequence number.

//Q16. Why do you split the distance and vibration into two bytes?
//Each byte can store only 8 bits. Splitting a larger number into two bytes allows the Arduino to send its lower and upper bytes separately.

//Q17. What is the sequence number used for?
//It helps identify the order of the data packets. It increases each time a packet is sent and wraps around after 255.


//Q18. What happens if communication fails?
//My code checks whether the sensor-data transmission succeeds and whether the expected four-byte fan status is received.
// If these checks fail, it can report a communication error rather than treating incomplete data as a valid response.
