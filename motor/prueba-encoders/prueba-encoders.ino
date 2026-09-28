const int A = 16;
const int B = 17;

void setup() {
  Serial.begin(115200);

  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
}

void loop() {
  Serial.print(digitalRead(A));
  Serial.print(" ");
  Serial.println(digitalRead(B));

  delay(50);
}