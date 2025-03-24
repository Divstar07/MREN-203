void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    int data = Serial.read() - '0';
    Serial.print("You sent me: ");
    Serial.println(data);
  }
}