#include <Servo.h>

enum MOTORSELECTION {
  NONE,
  LINEARACTUATOR,
  YAW,
  PITCH,
  LAUNCH,
} mov_mode = NONE;


Servo myActuator;  // create servo object to control the actuator
int minPos = 0;    // minimum position of the actuator in mm
int maxPos = 50;  // maximum position of the actuator in mm
bool synced = true; // flag to indicate if the actuator is in sync with the target position
int tolerance = 2; // tolerance range in mm
int targetPos = 0; // target position variable
int lastPos = 0;    // variable to store the last set position of the actuator

void linearActuator(){
  if (Serial.available() > 0) {
    targetPos = Serial.parseInt(); // read the input from the serial monitor
    if (targetPos >= minPos && targetPos <= maxPos) { // check if the target position is within limits
      synced = false; // set the synced flag to false to indicate a new target position
      Serial.println("Position set to: " + String(targetPos) + " mm");
    } else {
      Serial.println("Position out of bounds");
    }
    // Clear the serial buffer to prevent issues with subsequent inputs
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  if (!synced) {
    if (myActuator.readMicroseconds() != lastPos) {
      lastPos = myActuator.readMicroseconds();
      return;
    }

    int currentPos = map(lastPos, 1000, 2000, minPos, maxPos); // map the last position to the mm scale
    if (currentPos < targetPos - tolerance) {
      myActuator.writeMicroseconds(map(currentPos + 1, minPos, maxPos, 1000, 2000)); // move the actuator towards the target position
    } else if (currentPos > targetPos + tolerance) {
      myActuator.writeMicroseconds(map(currentPos - 1, minPos, maxPos, 1000, 2000)); // move the actuator towards the target position
    } else {
      synced = true; // set the synced flag to true once the actuator reaches the target position
    }
    delay(10); // small delay for smooth movement
  }  
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
      int pos = input.substring(input.indexOf(':') + 2).toInt(); // Extract the position value
      linearActuator(pos);
    } 
    else if (input.startsWith("Yaw")) {
      int pos = input.substring(input.indexOf(':') + 2).toInt(); // Extract the position value
      // Code to adjust yaw position using 'pos'
    }
  }
}



void setup() {
  Serial.begin(9600);   // initialize serial communication
  myActuator.attach(9); // attaches the servo on pin 9 to the servo object
  myActuator.writeMicroseconds(1500); // center the actuator
}

void loop() {
  linearActuator();
  //operateMotors();
}
