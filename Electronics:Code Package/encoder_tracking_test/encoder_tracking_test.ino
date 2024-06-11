#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

// Define encoder pins and properties
const int ArmEncoderPinA = A0;
const int ArmEncoderPinB = A1;
const float ArmEncoderCPR = 5.45; // Counts per revolution
float offset = 0.0;

// Create encoder instance
Encoder ArmEncoder = Encoder(ArmEncoderPinA, ArmEncoderPinB, ArmEncoderCPR);

// Interrupt routines for encoder
void doA() { ArmEncoder.handleA(); }
void doB() { ArmEncoder.handleB(); }

// Create PCI listeners for encoder
PciListenerImp listenerA(ArmEncoderPinA, doA);
PciListenerImp listenerB(ArmEncoderPinB, doB);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize encoder hardware
  ArmEncoder.init();

  // Register listeners for encoder pins
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  Serial.println("Arm Encoder ready");
  delay(1000);
}

void loop() {
  // Update the sensor values as frequently as possible
  ArmEncoder.update();

  // Display the angle to the terminal
  Serial.print("Angle: ");
  Serial.println(ArmEncoder.getAngle()-offset);
  Serial.print("Speed: ");
  Serial.println(ArmEncoder.getVelocity());
  Serial.println();
  
  // Check for serial input
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'r') {
      offset = ArmEncoder.getAngle();
      Serial.println("Encoder count reset to 0");
    }
  }

  // Add a small delay to reduce serial output frequency
  delay(100);
}
