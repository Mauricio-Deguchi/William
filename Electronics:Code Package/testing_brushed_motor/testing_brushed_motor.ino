#include <Servo.h>

Servo esc;  // create servo object to control the ESC

int throttlePin = 8;  // PWM pin connected to the ESC signal wire
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // Attach the ESC on pin 8
  Serial.begin(9600);
  esc.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  esc.writeMicroseconds(1500);
}

void loop() {
  // print the string when a newline arrives
  if (stringComplete) {
    // convert the string to an integer
    int speed = inputString.toInt();

    //then myFun(Val)
    esc.write(map(speed, 0, 100, 1000, 2000));
    Serial.println(speed);

    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  // read from the serial port
  while (Serial.available() > 0 && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag indicating the string is complete:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  delay(20); // Small delay to stabilize the servo
}
