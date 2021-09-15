#define PIN7 7

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN7, LOW);
  delay(1000);
  for (int i=0;i<=4;i++) //5회 반복
  {
    digitalWrite(PIN7, HIGH);
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);
  }
  digitalWrite(PIN7, HIGH);
  while(1){} //infinite loop
}
