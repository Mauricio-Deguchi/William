#include <Servo.h>
#include "CytronMotorDriver.h"
#include <Encoder.h>
#include <math.h>

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
const float Kp = 0.275;
const float Ki = 0.0005; // Integral gain
const float Kd = 0.01; // Derivative gain
const float degreeTolerance = 5;  // tolerance for the launch angle

float lastError = 0; // Last error for derivative calculation
float integral = 0; // Integral term

///////////////////////////////////////
// Define encoder pins and properties//
///////////////////////////////////////

#define ArmEncoderPinA 18
#define ArmEncoderPinB 19
const float ArmEncoderCPR = 1200.0; // Counts per revolution

// Create encoder instance
Encoder ArmEncoder(ArmEncoderPinA, ArmEncoderPinB);

// Variables to store the encoder counts and previous counts
long encoderCounts = 0;
long previousCounts = 0;

// Variables to store the degrees and speed
float ArmDegrees = 0.0;
float ArmSpeed = 0.0;

// Time variables for calculating speed
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long interval = 10; // Interval in milliseconds


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
    integral = 0;
    lastError = 0;
    rpm = 0;
    PWMspeed = 0;
    return;
  }

  const float maxIntegral = 1000; // Maximum integral value to prevent windup
  const unsigned long controlInterval = 1000; // Control loop interval in milliseconds
  unsigned long lastTime = millis();
  bool firstAttemptFlag = true;
  while (!BrushlessMotorSpeedReached) {
    if (millis() - lastTime >= controlInterval) {
      lastTime = millis();
      
      rpm = (float)duration / RPM_Scale; // Calculate RPM
      Serial.print("Boomerang RPM: ");
      Serial.print(rpm);
      duration = 0;
      float error = 2*tolerance;
      float derivative = 0;
      if(firstAttemptFlag)
      {
        firstAttemptFlag = false;
      }
      else
      {
        // Adjust motor speed based on desired RPM
        error = desiredRPM - rpm;
        integral += error * (controlInterval / 1000.0); // Accumulate the integral of the error
        integral = constrain(integral, -maxIntegral, maxIntegral); // Prevent integral windup
        derivative = (error - lastError) / (controlInterval / 1000.0);
        lastError = error; // Update lastError for the next iteration
      }
      
      if (abs(error) > tolerance) {
        // Calculate the new speed
        PWMspeed += Kp * error + Ki * integral + Kd * derivative;
        PWMspeed = constrain(PWMspeed, 0, 255); // Ensure PWMspeed is within valid range
      } else {
        BrushlessMotorSpeedReached = true;
      }
      
      Serial.print(" | Error: ");
      Serial.print(error);
      Serial.print(" | Derivative: ");
      Serial.print(derivative);
      Serial.print(" | Integral: ");
      Serial.print(integral);
      Serial.print(" | Signal: ");
      Serial.println(PWMspeed);

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
      float offset = ArmEncoder.readAndReset();
      Serial.print("Encoder count reset to 0, offset: ");
      Serial.println(offset);
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
  while (!doneTracking) {
    // Read the current encoder counts
    encoderCounts = ArmEncoder.read();

    // Calculate the degrees
    ArmDegrees = fmod((encoderCounts / ArmEncoderCPR) * 360.0, 360.0);

    // Ensure degrees is positive
    if (ArmDegrees < 0) {
      ArmDegrees += 360.0;
    }
    Serial.print("Degrees: ");
    Serial.print(ArmDegrees);
    Serial.print(" | ");
    // Get the current time
    currentTime = millis();

    // Calculate the speed every interval
    if (currentTime - previousTime >= interval) {
      // Calculate the speed in degrees per second
      ArmSpeed = abs(((encoderCounts - previousCounts) / ArmEncoderCPR) * 360.0 / ((currentTime - previousTime) / 1000.0));

      // Update previous counts and previous time
      previousCounts = encoderCounts;
      previousTime = currentTime;

      Serial.print("Speed: ");
      Serial.println(ArmSpeed/6);

      // Check if the current RPM is equal or greater than the target RPM
      if (ArmSpeed/6 >= targetRPM) {

        // Calculate the difference to the release angle
        float angleDifference = fabs(ArmDegrees - releaseAngle);
        if (angleDifference < degreeTolerance) {
          // Call the solenoid control function
          controlSolenoid();
          doneTracking = true; // Stop tracking once solenoid is activated
        }
      }
    }

    delay(10); // Small delay for smooth updates
  }
  Serial.println("Exiting Arm Encoder Tracker");
}

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  myActuator.attach(9); // Attach the servo on pin 9 to the servo object
  pinMode(SolenoidPin, OUTPUT); // Set the solenoid pin as output
  myActuator.writeMicroseconds(1500); // Center the actuator
  ArmEncoder.write(0); // Reset the encoder

  // Register listeners for encoder pins

  Serial.println("Arm Encoder ready");
  delay(1000);
  EncoderInit(); // Initialize wheel encoder
}

void loop() {
  operateMotors();
}
