const int Solenoid = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Solenoid, OUTPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(Solenoid, HIGH);
  delay(3000);
  digitalWrite(Solenoid, LOW);
  delay(3000);
  //delay(3000);
  //delay(2000);
  //digitalWrite(Solenoid, LOW);
  //delay(4000);
  //delay(100);
  //digitalWrite(Solenoid, LOW);
  //for (int i = 1; i<=10; i++){
    //delay(1000);
    //Serial.println(i);
  //}
}
