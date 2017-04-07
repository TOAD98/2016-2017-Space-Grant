void setup() {
  // put your setup code here, to run once:
  pinMode(12,OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(A0)/4);
  analogWrite(9, analogRead(A0)/4);
  delay(10);
}
