void setup() {
  delay(100);
  Serial.begin(115200);
  pinMode(0, OUTPUT);
}

void loop() {
  delay(10);
  //Serial.println(F("Debug 1"));
  digitalWrite(0, HIGH);
  delay(2000);
  //Serial.println("Debug 2");
  digitalWrite(0, LOW);
  delay(1000);
  //Serial.println("Debug 3");
}
