#include <Servo.h>
#include "CytronMotorDriver.h"

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
const int tolerance = 3; // RPM
const float Kp = 0.5;
const float Kd = 0.05; // Derivative gain

float lastError = 0; // Last error for derivative calculation

void linearActuator(float targetPos) {
  synced = false; // flag to indicate if the actuator is in sync with the target position

  // we will iterate through this loop giving position feedback 
  // unit we reach the desired linear actuator position
  while (!synced) {
    // first, we'
    if (myActuator.readMicroseconds() != lastPos) {
      lastPos = myActuator.readMicroseconds();
    }

    // the map function will take our position reading from the linear actuator 
    //and converts it to the mm scale defined by the minPos and maxPos
    float currentPos = map(lastPos, 1000, 2000, minPos, maxPos); // map the last position to the mm scale

    // if the current position is less than the target position, we need to extend the actuator
    if (currentPos < targetPos - tolerance_linear) {
      myActuator.writeMicroseconds(map(currentPos + 1, minPos, maxPos, 1000, 2000)); // move the actuator towards the target position
    } else if (currentPos > targetPos + tolerance_linear) {
      // otherwise, we'll want to contract the actuator
      myActuator.writeMicroseconds(map(currentPos - 1, minPos, maxPos, 1000, 2000)); // move the actuator towards the target position
    } else {
      synced = true; // set the synced flag to true once the actuator reaches the target position
    }
    delay(10); // small delay for smooth movement
  }
}

void controlBrushedMotor(float desiredRPM) {
  // Use the servo library to control the brushless motor
  // Note: Controlling a brushless motor typically requires an ESC
  // and is not as simple as using a servo library, but this is a simplified example
  bool BrushlessMotorSpeedReached = false;
  while(!BrushlessMotorSpeedReached){
    if (millis() - lastTime >= 1000) {
      rpm = (float)duration/RPM_Scale; // Calculate RPM
      Serial.print("RPM:");
      Serial.println(rpm);
      duration = 0;
      lastTime = millis();
  
      // Adjust motor speed based on desired RPM
      float error = desiredRPM - rpm;
      float derivative = error - lastError;
      
      if (abs(error) > tolerance) {
        // we need to adjust the speed
        if (rpm < desiredRPM){
          // Increase motor speed
          PWMspeed -= Kp*abs(error) + Kd * derivative;  
        }
       else if (rpm > desiredRPM) {
        // Decrease motor speed
        PWMspeed += Kp*abs(error)+ Kd * derivative;
        }
      }
      else
      {
        BrushlessMotorSpeedReached = true;
      }
      motor.setSpeed(PWMspeed); 
    }
  }
  Serial.println("Exiting Brushed Motors");
}

void controlSolenoid() {
  digitalWrite(SolenoidPin, HIGH);
  delay(50);
  digitalWrite(SolenoidPin, LOW);
}

void operateMotors() {
  // This function will serve as a catch-all to serve as communication between Python and the Arduino
  // The objective of this function is to serve as the control center for which motor we want to adjust
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input string until newline character
    input.trim(); // Remove any leading or trailing whitespace
    Serial.println("Received: " + input); // Print the received string for debugging

    // Parse the input string and perform actions accordingly
    if (input.startsWith("Linear Actuator")) {
      Serial.println("Proceeding to Linear actuator");
      float pos = input.substring(input.indexOf(':') + 2).toFloat(); // Extract the position value
      linearActuator(pos);
    } 
    else if (input.startsWith("Yaw")) { 
      int pos = input.substring(input.indexOf(':') + 2).toInt(); // Extract the position value
      Serial.println("Proceeding to Yaw"); // I haven't made a yaw function yet
    }
    else if (input.startsWith("Brushed Motor")) {
      int speed = input.substring(input.indexOf(':') + 2).toInt(); // Extract the speed value
      Serial.println("Proceeding to Brushed Motor");
      controlBrushedMotor(speed);
    }
    else if (input.startsWith("Fire Solenoids")) {
      Serial.println("Firing Solenoids");
      controlSolenoid();
    }
    else if (input.startsWith("Track Encoder")) {
      Serial.println("Tracking Encoder");
      float releaseAngle = input.substring(input.indexOf("Release Angle:") + 14).toFloat();
      float targetRPM = input.substring(input.indexOf("Target RPM:") + 11).toFloat();
      trackEncoder(releaseAngle, targetRPM);
    }
    else if (input.startsWith("Reset Encoder")) {
      ArmEncoderPos = 0;
      Serial.println("Encoder count reset to 0");
    }
  }
}


void EncoderInit() {
  Direction = true; // default -> Forward
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
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

void setup() {
  Serial.begin(9600);   // initialize serial communication
  myActuator.attach(9); // attaches the servo on pin 9 to the servo object
  pinMode(SolenoidPin, OUTPUT); // set the solenoid pin as output
  myActuator.writeMicroseconds(1500); // center the actuator
  EncoderInit(); // Initialize the module
}

void loop() {
  operateMotors();
}
