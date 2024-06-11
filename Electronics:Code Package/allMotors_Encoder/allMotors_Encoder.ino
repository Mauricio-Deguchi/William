#include <Servo.h>
#include "CytronMotorDriver.h"
#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

// Create instances for actuators and motor driver
Servo myActuator;       // create servo object to control the actuator
CytronMD motor(PWM_DIR, 3, 5);  // PWM = Pin 3, DIR = Pin 5.

const int SolenoidPin = 10;  // define the pin for the solenoid

static float minPos = 0.0;    // minimum position of the actuator in mm
static float maxPos = 50.0;   // maximum position of the actuator in mm
bool synced;                   // flag to indicate if the actuator is in sync with the target position
static float tolerance_linear = 0.1; // tolerance range in mm for linear actuator
float targetPos = 0.0;        // target position variable
float lastPos = 0.0;          // variable to store the last set position of the actuator
bool solenoid = false;
const byte encoder0pinA = 2; // A pin -> the interrupt pin 0
const byte encoder0pinB = 4; // B pin -> the digital pin 4
byte encoder0PinALast;
volatile int duration; // the number of pulses
boolean Direction; // the rotation direction
unsigned long lastTime = 0;
float rpm = 0;
float desiredRPM = 0;
int PWMspeed = 0;
const float RPM_Scale = 10.67; // RPM scale factor
const int tolerance = 3; // RPM tolerance
const float Kp = 0.5;
const float Ki = 0.1; // Integral gain
const float Kd = 0.05; // Derivative gain
const float degreeTolerance = 5;  // tolerance for the launch angle

float lastError = 0; // Last error for derivative calculation
float integral = 0; // Integral term

///////////////////////////////////////
// Define encoder pins and properties//
///////////////////////////////////////
const int ArmEncoderPinA = A0;
const int ArmEncoderPinB = A1;
const float ArmEncoderCPR = 8192.0; // Counts per revolution
float offset = 0.0;

// Create encoder instance
Encoder ArmEncoder = Encoder(ArmEncoderPinA, ArmEncoderPinB, ArmEncoderCPR);

// Interrupt routines for encoder
void doA() { ArmEncoder.handleA(); }
void doB() { ArmEncoder.handleB(); }

// Create PCI listeners for encoder
PciListenerImp listenerA(ArmEncoderPinA, doA);
PciListenerImp listenerB(ArmEncoderPinB, doB);

void linearActuator(float targetPos) {
  synced = false; // flag to indicate if the actuator is in sync with the target position

  // Iterate through this loop giving position feedback until we reach the desired linear actuator position
  while (!synced) {
    if (myActuator.readMicroseconds() != lastPos) {
      lastPos = myActuator.readMicroseconds();
    }

    // Map the last position to the mm scale
    float currentPos = map(lastPos, 1000, 2000, minPos, maxPos);

    // Move the actuator towards the target position
    if (currentPos < targetPos - tolerance_linear) {
      myActuator.writeMicroseconds(map(currentPos + 1, minPos, maxPos, 1000, 2000));
    } else if (currentPos > targetPos + tolerance_linear) {
      myActuator.writeMicroseconds(map(currentPos - 1, minPos, maxPos, 1000, 2000));
    } else {
      synced = true; // Set the synced flag to true once the actuator reaches the target position
    }

    Serial.print("Position: ");
    Serial.print(currentPos);
    Serial.println(" mm");
    delay(10); // Small delay for smooth movement
  }
  Serial.println("Exiting Linear actuator");
}

void controlBrushedMotor(float desiredRPM) {
  bool BrushlessMotorSpeedReached = false;
  if(desiredRPM == 0){
    motor.setSpeed(0); 
    return;
  }
  while (!BrushlessMotorSpeedReached) {
    if (millis() - lastTime >= 1000) {
      rpm = (float)duration / RPM_Scale; // Calculate RPM
      Serial.print("Boomerang RPM: ");
      Serial.println(rpm);
      duration = 0;
      lastTime = millis();
  
      // Adjust motor speed based on desired RPM
      float error = desiredRPM - rpm;
      integral += error; // Accumulate the integral of the error
      float derivative = error - lastError;
      lastError = error; // Update lastError for the next iteration
      
      if (abs(error) > tolerance) {
        // Adjust the speed
        if (rpm < desiredRPM) {
          // Increase motor speed
          PWMspeed += Kp * error + Ki * integral + Kd * derivative;  
        } else if (rpm > desiredRPM) {
          // Decrease motor speed
          PWMspeed -= Kp * error + Ki * integral + Kd * derivative;
        }
      } else {
        BrushlessMotorSpeedReached = true;
      }
      motor.setSpeed(PWMspeed); 
    }
  }
  Serial.println("Exiting Brushed Motors");
}

void controlSolenoid() {
  digitalWrite(SolenoidPin, HIGH);
  delay(100);
  digitalWrite(SolenoidPin, LOW);
  Serial.println("Exiting Solenoids");
}

void operateMotors() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input string until newline character
    input.trim(); // Remove any leading or trailing whitespace
    Serial.println("Received: " + input); // Print the received string for debugging

    // Parse the input string and perform actions accordingly
    if (input.startsWith("Linear Actuator")) {
      Serial.println("Proceeding to Linear actuator");
      float pos = input.substring(input.indexOf(':') + 2).toFloat(); // Extract the position value
      linearActuator(pos);
    } else if (input.startsWith("Yaw")) { 
      int pos = input.substring(input.indexOf(':') + 2).toInt(); // Extract the position value
      Serial.println("Proceeding to Yaw"); // Yaw function placeholder
    } else if (input.startsWith("Brushed Motor")) {
      int speed = input.substring(input.indexOf(':') + 2).toInt(); // Extract the speed value
      Serial.println("Proceeding to Brushed Motor");
      controlBrushedMotor(speed);
    } else if (input.startsWith("Fire Solenoids")) {
      Serial.println("Firing Solenoids");
      controlSolenoid();
    } else if (input.startsWith("Track Encoder")) {
      Serial.println("Tracking Encoder");
      float releaseAngle = input.substring(input.indexOf("Release Angle:") + 14).toFloat();
      float targetRPM = input.substring(input.indexOf("Target RPM:") + 11).toFloat();
      trackEncoder(releaseAngle, targetRPM);
    } else if (input.startsWith("Reset Encoder")) {
      offset = ArmEncoder.getAngle();
      Serial.println("Encoder count reset to 0");
    }
  }
}

void EncoderInit() {
  Direction = true; // Default -> Forward
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeed, CHANGE);
}

void wheelSpeed() {
  int Lstate = digitalRead(encoder0pinA);
  if ((encoder0PinALast == LOW) && Lstate == HIGH) {
    int val = digitalRead(encoder0pinB);
    if (val == LOW && Direction) {
      Direction = false; // Reverse
    } else if (val == HIGH && !Direction) {
      Direction = true; // Forward
    }
  }
  encoder0PinALast = Lstate;

  if (!Direction)
    duration++;
  else
    duration--;
}

// Arm Encoder
void trackEncoder(float releaseAngle, float targetRPM) {
  bool doneTracking = false;
  float degrees = 0;
  float ArmSpeed = 0;
  float scalingFactor = 5.8827;
  while (!doneTracking) {
    ArmEncoder.update();

    // Display the angle to the terminal
    degrees = ArmEncoder.getAngle() - offset;
    ArmSpeed = ArmEncoder.getVelocity()/scalingFactor;
    Serial.print("Speed: ");
    Serial.println(ArmSpeed);

    // Check if the current RPM is equal or greater than the target RPM
    if (ArmSpeed >= targetRPM) {
      // Normalize degrees within 0 to 360
      Serial.println("Target RPM Reached");
      float normalizedDegrees = degrees - 360.0 * fmod(degrees, 360.0);
      
      // Calculate the difference to the release angle
      float angleDifference = fabs(normalizedDegrees - releaseAngle);
      if (angleDifference < degreeTolerance) {
        // Call the solenoid control function
        controlSolenoid();
        doneTracking = true; // Stop tracking once solenoid is activated
      }
      Serial.print("Angle Difference: ");
      Serial.println(angleDifference);
    }

    delay(10); // Update every 100ms
  }
  Serial.println("Exiting Arm Encoder Tracker");
}

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  myActuator.attach(9); // Attach the servo on pin 9 to the servo object
  pinMode(SolenoidPin, OUTPUT); // Set the solenoid pin as output
  myActuator.writeMicroseconds(1500); // Center the actuator
  ArmEncoder.init();

  // Register listeners for encoder pins
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  Serial.println("Arm Encoder ready");
  delay(1000);
  EncoderInit(); // Initialize wheel encoder
}

void loop() {
  operateMotors();
}
