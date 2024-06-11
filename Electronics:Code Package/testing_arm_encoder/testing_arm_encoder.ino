#include <Encoder.h>

// Define the pins connected to the encoder
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// Create an Encoder object
Encoder ArmEncoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Encoder CPR (Counts Per Revolution)
const float ArmEncoderCPR = 1200.0;

// Variables to store the encoder counts and previous counts
long encoderCounts = 0;
long previousCounts = 0;

// Variables to store the degrees and speed
float degrees = 0.0;
float speed = 0.0;

// Time variables for calculating speed
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long interval = 100; // Interval in milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize previousTime
  previousTime = millis();
}

void loop() {
  // Read the current encoder counts
  encoderCounts = ArmEncoder.read();

  // Calculate the degrees
  degrees = (encoderCounts / ArmEncoderCPR) * 360.0;

  // Get the current time
  currentTime = millis();

  // Calculate the speed every interval
  if (currentTime - previousTime >= interval) {
    // Calculate the speed in degrees per second
    speed = ((encoderCounts - previousCounts) / ArmEncoderCPR) * 360.0 / ((currentTime - previousTime) / 1000.0);

    // Update previous counts and previous time
    previousCounts = encoderCounts;
    previousTime = currentTime;
  }

  // Print the degrees and speed to the serial monitor
  Serial.print("Degrees: ");
  Serial.print(degrees);
  Serial.print(" | Speed: ");
  Serial.print(speed);
  Serial.println(" degrees/second");

  // Add a small delay to avoid overwhelming the serial monitor
  delay(10);
}
