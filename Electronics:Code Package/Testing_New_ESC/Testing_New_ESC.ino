#include <VescUart.h>
#include <SoftwareSerial.h>

VescUart vesc;
SoftwareSerial vescSerial(2, 3); // RX, TX

int targetSpeed = 0; // Target speed in RPM
int currSpeed = 0; // Current speed in RPM
float current = 0;
int tolerance = 50; // Velocity tolerance in rpm
float Kp = 0.0001;         // Proportional controller

void setup() {
  Serial.begin(9600);
  vescSerial.begin(19200);
  vesc.setSerialPort(&vescSerial);
}

void loop() {
  if (Serial.available() > 0) {
    targetSpeed = Serial.parseInt(); // Read the target speed from the serial monitor
    if (targetSpeed != currSpeed) {
      setTargetSpeed(targetSpeed); // Set the target speed if it's different from the last value
      currSpeed = targetSpeed; // Update the last target speed
    }
  }

  if (vesc.getVescValues()) {
    int actualSpeed = vesc.data.rpm; // Read the actual speed from the VESC
    Serial.print("Actual Speed: ");
    Serial.print(actualSpeed);
    Serial.println(" RPM");
  }

  delay(1000);
}

void setTargetSpeed(int speed) {
  Serial.print("Setting Target Speed: ");
  Serial.print(speed);
  Serial.println("RMP");
  while (abs(targetSpeed - currSpeed) > tolerance)
  {
    Serial.print("Error: ");
    Serial.println(targetSpeed - currSpeed);
    current = current + Kp * (targetSpeed - currSpeed);
    vesc.setCurrent(current); // Set the target speed using setCurrent() function 
    delay(1000); 
    vesc.getVescValues();
    int actualSpeed = vesc.data.rpm;
    currSpeed = actualSpeed;
    Serial.print("Current: ");
    Serial.println(current);
    Serial.println();
    Serial.print("Current Speed: ");
    Serial.println(currSpeed);
  }
}
