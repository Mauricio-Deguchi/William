#include "CytronMotorDriver.h"
// Configure the motor driver.
CytronMD motor(PWM_DIR, 3, 5);  // PWM = Pin 3, DIR = Pin 5.

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

void setup() {
  Serial.begin(9600); // Initialize the serial port
  EncoderInit(); // Initialize the module
}

void loop() {
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
    motor.setSpeed(PWMspeed); 
  }

  // Check for user input for desired RPM
  if (Serial.available() > 0) {
    desiredRPM = Serial.parseFloat(); // Read the desired RPM value from the serial port
    Serial.print("Desired RPM set to: ");
    Serial.println(desiredRPM);
    if(desiredRPM == 0.0){
      Serial.println("Setting Speed to zero");
      rpm = 0;
      PWMspeed = 0;
      motor.setSpeed(PWMspeed);
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
