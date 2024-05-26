// Import necessary libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Initialize the accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// Define pin numbers for step and direction control for X, Y, and Z axes
const int stepX = 2;
const int dirX  = 5;

const int stepY = 3;
const int dirY  = 6;

const int stepZ = 4;
const int dirZ  = 7;

// Enable pin for the stepper motors
const int enPin = 8;

// Variable to keep track of ticks (loop iterations)
unsigned int ticks = 0;

// Variables for position and direction control
int x = 0;
int y = 0;
int xDir = 0; // 0 for backwards, 1 for forwards
int yDir = 0; // 0 for backwards, 1 for forwards

// Variables for sensor data
int sensorX = 0;
int sensorY = 0;

// Variables for new and actual positions
int newX = 0;
int newY = 0;
int trueX = 0;
int trueY = 0;

// Additional variables for logic control
int a = 1;
int b = 2;

bool pauseX = false;
bool pauseY = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Check if the accelerometer sensor is detected
  if (!accel.begin()) {
    Serial.println("No valid sensor found");
    while (1); // Stop execution if no sensor is found
  }

  // Set pin modes for step and direction pins as outputs
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);

  pinMode(stepY, OUTPUT);
  pinMode(dirY, OUTPUT);

  pinMode(stepZ, OUTPUT);
  pinMode(dirZ, OUTPUT);

  // Set enable pin as output and activate it
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);

  // Initialize direction of the stepper motors
  digitalWrite(dirX, LOW);
  digitalWrite(dirY, LOW);
  digitalWrite(dirZ, HIGH);
}

void loop() {
  // Perform actions every 100 ticks
  if (ticks % 100 == 0) {
    sensors_event_t event;
    accel.getEvent(&event); // Get sensor data

    // Print sensor data and positions to the serial monitor
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("NEW x pos: "); Serial.print(newX); Serial.print("  ");
    Serial.print("ACTUAL x pos: "); Serial.print(trueX); Serial.print("  ");
    Serial.println(xDir);

    // Map accelerometer data to new range and calculate sensor positions
    event.acceleration.x = map(event.acceleration.x, -10, 10, -5, 5);
    event.acceleration.y = map(event.acceleration.y, -10, 10, -5, 5);

    sensorX = event.acceleration.x * 10;
    sensorY = event.acceleration.y * 10;

    // Update new positions based on sensor data
    newX += sensorX;
    newY += sensorY;
  }

  // Determine direction based on new and actual positions
  xDir = (newX - trueX > 0) ? 1 : -1;
  yDir = (newY - trueY > 0) ? 1 : -1;

  // Check if movements should be paused
  pauseX = newX == trueX;
  pauseY = newY == trueY;

  // Move stepper motor if not paused
  if (!pauseX) {
    step(stepX, xDir, dirX);
    trueX += (xDir == 1 ? 1 : -1);
  }

  if (!pauseY) {
    step(stepY, yDir, dirY);
    trueY += (yDir == 1 ? 1 : -1);
  }

  // Increment ticks
  ticks++;
}

// Function to control a single step of a stepper motor
void step(int pin, int dirpin, int posDir) {
  int _delay = 500;

  // Set direction of the stepper motor
  if (dirpin == 1) {
    digitalWrite(posDir, LOW);
  } else {
    digitalWrite(posDir, HIGH);
  }

  // Perform a single step
  digitalWrite(pin, HIGH);
  delayMicroseconds(_delay);
  digitalWrite(pin, LOW);
  delayMicroseconds(_delay);
}

// Placeholder function for future implementation
void complete(int pin, int dirpin, int posDir, int truePos, int newPos) {
  // Currently not implemented
}
