/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>
#include <SoftwareSerial.h>

/** Initiate VescUart class */
VescUart vesc;
SoftwareSerial vescSerial(2, 3); // RX, TX

float speed = 0.0; /** The current in amps */
float poles = 14.0; // number of poles in the brushless motor
float gearRatio = 3.75;

void setup() {
  Serial.begin(9600);
  /** Setup UART port (Serial1 on Atmega32u4) */
  vescSerial.begin(19200);
  while (!vescSerial) {;}
  /** Define which ports to use as UART */
  vesc.setSerialPort(&vescSerial);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the input from the serial monitor
    speed = Serial.parseFloat();
    Serial.print("Recieved: ");
    Serial.println(speed);
    speed = speed*poles*gearRatio;
    // Wait for a brief moment to allow the input to be processed
    delay(100);
  }
  
  /** Call the function setCurrent() to set the motor current */
  vesc.setRPM(speed);
  vesc.getVescValues();
  float actualSpeed = vesc.data.rpm;
  Serial.print("Speed: ");
  Serial.println(actualSpeed/poles/gearRatio);
  // UART.setBrakeCurrent(current);
  
}
