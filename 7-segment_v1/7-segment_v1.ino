#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 objects
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First PCA9685
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second PCA9685

// Pin Definitions
const int rightIrSensorPin = 2;  // Right IR sensor connected to pin 2
const int leftIrSensorPin = 5;   // Left IR sensor connected to pin 5
const int ledPin = 4;            // LED connected to pin 4

// Variables for Timing and State Management
unsigned long detectionTime = 0;  // Stores the time when an object is detected
bool objectDetected = false;      // Tracks if an object is currently detected
int randomNumber = 0;             // Stores the generated random number

// Lookup table for segment angles (ON and OFF)
const int segmentAngles[10][7] = {
  // A, B, C, D, E, F, G (ON angles for each segment)
  {90, 90, 90, 90, 90, 90,  0}, // Digit 0
  { 0, 90, 90,  0,  0,  0,  0}, // Digit 1
  {90, 90,  0, 90, 90,  0, 90}, // Digit 2
  {90, 90, 90, 90,  0,  0, 90}, // Digit 3
  { 0, 90, 90,  0,  0, 90, 90}, // Digit 4
  {90,  0, 90, 90,  0, 90, 90}, // Digit 5
  {90,  0, 90, 90, 90, 90, 90}, // Digit 6
  {90, 90, 90,  0,  0,  0,  0}, // Digit 7
  {90, 90, 90, 90, 90, 90, 90}, // Digit 8
  {90, 90, 90,  0,  0, 90, 90}  // Digit 9
};

// Servo channel assignments
const int digit1ServoChannels[7] = {0, 1, 2, 3, 4, 5, 6};  // Channels for digit 1 (A-G)
const int digit2ServoChannels[7] = {7, 8, 9, 10, 11, 12, 13}; // Channels for digit 2 (A-G)
const int digit3ServoChannels[7] = {14, 15, 0, 1, 2, 3, 4};   // Channels for digit 3 (A-G)
const int minusSignServoChannel = 5; // Channel for minus sign servo

// Function to set servo angle
void setServoAngle(int channel, int angle) {
  if (channel < 16) {
    pwm1.setPWM(channel, 0, angleToPulse(angle)); // Control servos on first PCA9685
  } else {
    pwm2.setPWM(channel - 16, 0, angleToPulse(angle)); // Control servos on second PCA9685
  }
}

// Function to convert angle to PWM pulse
int angleToPulse(int angle) {
  return map(angle, 0, 180, 150, 600); // Convert angle (0-180) to PWM pulse (150-600)
}

// Function to display a digit on a specific 7-segment display
void displayDigit(int digit, const int servoChannels[7]) {
  for (int segment = 0; segment < 7; segment++) {
    setServoAngle(servoChannels[segment], segmentAngles[digit][segment]);
  }
}

// Function to display the minus sign
void displayMinusSign(bool state) {
  setServoAngle(minusSignServoChannel, state ? 90 : 0); // 90° for ON, 0° for OFF
}

// Function to calibrate servos at startup
void calibrateServos() {
  // Move all servos to OFF position
  for (int segment = 0; segment < 7; segment++) {
    setServoAngle(digit1ServoChannels[segment], 0);
    setServoAngle(digit2ServoChannels[segment], 0);
    setServoAngle(digit3ServoChannels[segment], 0);
  }
  displayMinusSign(false); // Turn off minus sign
  delay(1000); // Wait for 1 second

  // Test each digit
  for (int digit = 0; digit < 10; digit++) {
    displayDigit(digit, digit1ServoChannels); // Display digit on digit 1
    delay(1000); // Wait for 1 second
  }

  // Return all servos to OFF position
  for (int segment = 0; segment < 7; segment++) {
    setServoAngle(digit1ServoChannels[segment], 0);
    setServoAngle(digit2ServoChannels[segment], 0);
    setServoAngle(digit3ServoChannels[segment], 0);
  }
  displayMinusSign(false); // Turn off minus sign
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize PCA9685 boards
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(50); // Set PWM frequency to 50Hz
  pwm2.setPWMFreq(50);

  // Configure IR Sensor Pins with Internal Pull-Up Resistors
  pinMode(rightIrSensorPin, INPUT_PULLUP);
  pinMode(leftIrSensorPin, INPUT_PULLUP);

  // Configure LED Pin as Output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure LED is off initially

  // Seed the Random Number Generator
  randomSeed(analogRead(0));

  // Perform calibration
  calibrateServos();
}

void loop() {
  // Read the Right IR Sensor State (Active Low: LOW means object detected)
  int rightSensorState = digitalRead(rightIrSensorPin);

  // Read the Left IR Sensor State (Active Low: LOW means object detected)
  int leftSensorState = digitalRead(leftIrSensorPin);

  // Check if the right IR sensor detects an object and no object is currently being processed
  if (rightSensorState == LOW && !objectDetected) {
    // Object detected by the right IR sensor
    objectDetected = true;               // Set detection flag
    detectionTime = millis();            // Record the current time
    randomNumber = random(0, 1000);      // Generate a random number between 0 and 999
    digitalWrite(ledPin, HIGH);          // Turn on the LED

    // Print the detection message and random number to the Serial Monitor
    Serial.print("Right IR Sensor: Object Detected! Random Number: ");
    Serial.println(randomNumber);

    // Display the random number on the 7-segment display
    displayNumber(randomNumber);
  }

  // Check if the left IR sensor detects an object and no object is currently being processed
  if (leftSensorState == LOW && !objectDetected) {
    // Object detected by the left IR sensor
    objectDetected = true;               // Set detection flag
    detectionTime = millis();            // Record the current time
    randomNumber = -random(1, 1000);     // Generate a random number between -999 and -1
    digitalWrite(ledPin, HIGH);          // Turn on the LED

    // Print the detection message and random number to the Serial Monitor
    Serial.print("Left IR Sensor: Object Detected! Random Number: ");
    Serial.println(randomNumber);

    // Display the random number on the 7-segment display
    displayNumber(randomNumber);
  }

  // Check if 3 seconds have passed since the object was detected
  if (objectDetected && (millis() - detectionTime >= 3000)) {
    objectDetected = false;              // Reset detection flag
    digitalWrite(ledPin, LOW);           // Turn off the LED
    Serial.println("0");                 // Print "0" to the Serial Monitor
  }
}

// Function to display a number on the 7-segment display
void displayNumber(int number) {
  bool isNegative = (number < 0); // Check if the number is negative
  number = abs(number); // Work with the absolute value of the number

  // Split the number into digits
  int hundreds = number / 100;
  int tens = (number % 100) / 10;
  int units = number % 10;

  // Display the minus sign (if applicable)
  if (isNegative) {
    displayMinusSign(true);
  } else {
    displayMinusSign(false);
  }

  // Display the digits sequentially with a 500ms gap
  displayDigit(units, digit3ServoChannels); // Display unit's place
  delay(500);
  if (number >= 10) {
    displayDigit(tens, digit2ServoChannels); // Display ten's place
    delay(500);
  }
  if (number >= 100) {
    displayDigit(hundreds, digit1ServoChannels); // Display hundred's place
    delay(500);
  }
}