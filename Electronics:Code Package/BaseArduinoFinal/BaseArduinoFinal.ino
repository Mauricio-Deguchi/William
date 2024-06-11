#include "CytronMotorDriver.h"
#include <VescUart.h>
#include <SoftwareSerial.h>

// Configure the motor driver.
CytronMD motor(PWM_DIR, 4, 5);  // PWM = Pin 4, DIR = Pin 5.

/** Initiate VescUart class */
VescUart vesc;
SoftwareSerial vescSerial(2, 3); // RX, TX

float poles = 14.0; // number of poles in the brushless motor
float ArmGearRatio = 1.875;

enum MotorState {
  IDLE,
  CHANGING_SPEED,
  RUNNING
};

MotorState motorState = IDLE;
float currentRPM = 0.0;
float targetRPM = 0.0;
float stepRPM = 0.0;
unsigned long lastUpdateTime = 0;
unsigned long interval = 5000; // Total duration of the speed change in milliseconds
int steps = 10;  // Number of steps for the speed change

String Yaw(float angle) {
  // Placeholder for yaw function
  return "Yaw not implemented";
}

void changeSpeed(float newRPM) {
  targetRPM = newRPM;
  stepRPM = (targetRPM - currentRPM) / steps;
  motorState = CHANGING_SPEED;
  lastUpdateTime = millis();
}

void updateMotor() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= interval / steps) {
    lastUpdateTime = currentTime;
    if (motorState == CHANGING_SPEED) {
      currentRPM += stepRPM;
      if ((stepRPM > 0 && currentRPM >= targetRPM) || (stepRPM < 0 && currentRPM <= targetRPM)) {
        currentRPM = targetRPM;
        motorState = RUNNING;
      }
      vesc.setRPM(currentRPM);
    }
    vesc.getVescValues();
    float actualSpeed = vesc.data.rpm;
    Serial.print("Speed: ");
    Serial.print(actualSpeed / poles / ArmGearRatio);
    Serial.print("\t");
    Serial.println(currentRPM);
  }
}

void operateMotors() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input string until newline character
    input.trim(); // Remove any leading or trailing whitespace
    Serial.println("Received: " + input); // Print the received string for debugging

    // Parse the input string and perform actions accordingly
    if (input.startsWith("Yaw")) {
      int pos = input.substring(input.indexOf(':') + 2).toInt(); // Extract the position value
      Serial.println("Proceeding to Yaw"); // I haven't made a yaw function yet
      String result = Yaw(pos);
      Serial.println("Exiting Yaw");
    } else if (input.startsWith("Brushless Motor")) {
      int speed = input.substring(input.indexOf(':') + 2).toInt(); // Extract the speed value
      Serial.println("Desired Speed: " + String(speed));
      float newRPM = speed * poles * ArmGearRatio;
      changeSpeed(newRPM);
    } else {
      Serial.println("Unrecognized command");
    }
  }
}

void setup() {
  Serial.begin(9600);
  vescSerial.begin(19200);
  vesc.setSerialPort(&vescSerial);
  Serial.println("Base Arduino is ready");
}

void loop() {
  operateMotors();
  updateMotor();
}