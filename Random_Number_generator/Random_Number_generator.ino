/*
  Arduino Sketch for Dual IR Obstacle Avoidance Sensors with LED and Serial Output
  - Right IR Sensor (Active Low) connected to pin 2.
  - Left IR Sensor (Active Low) connected to pin 5.
  - LED connected to pin 4.
  - When the right IR sensor detects an object, a random number between 0 and 999 is generated.
  - When the left IR sensor detects an object, a random number between -999 and -1 is generated.
  - The random number and "Object Detected!" message are displayed on the Serial Monitor for 3 seconds.
  - During the 3-second window, new detections are ignored.
  - After 3 seconds, the Serial Monitor displays "0", and the system resumes monitoring.
  - The LED turns on when either sensor detects an object and stays on for 3 seconds.
*/

// Pin Definitions
const int rightIrSensorPin = 2;  // Right IR sensor connected to pin 2
const int leftIrSensorPin = 5;   // Left IR sensor connected to pin 5
const int ledPin = 4;            // LED connected to pin 4

// Variables for Timing and State Management
unsigned long detectionTime = 0;  // Stores the time when an object is detected
bool objectDetected = false;      // Tracks if an object is currently detected
int randomNumber = 0;             // Stores the generated random number

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Configure IR Sensor Pins with Internal Pull-Up Resistors
  pinMode(rightIrSensorPin, INPUT_PULLUP);
  pinMode(leftIrSensorPin, INPUT_PULLUP);

  // Configure LED Pin as Output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Ensure LED is off initially
 
  // Seed the Random Number Generator
  randomSeed(analogRead(0));
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
  }

  // Check if 3 seconds have passed since the object was detected
  if (objectDetected && (millis() - detectionTime >= 3000)) {
    objectDetected = false;              // Reset detection flag
    digitalWrite(ledPin, LOW);           // Turn off the LED
    Serial.println("0");                 // Print "0" to the Serial Monitor
  }

  // If an object is currently detected, keep printing the random number
  if (objectDetected) {
    if (randomNumber >= 0) {
      Serial.print("Right IR Sensor: Object Detected! Random Number: ");
    } else {
      Serial.print("Left IR Sensor: Object Detected! Random Number: ");
    }
    Serial.println(randomNumber);
  }
}