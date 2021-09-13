#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello world!");
  while (!Serial){
    ; //wait for serial port to connect
  }
  
  count=toggle=0;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  Serial.println(++count); 
  toggle=toggle_state(toggle);  //toggle LED value
  digitalWrite(PIN_LED, toggle); //update LED status
  delay(1000);
}

int toggle_state(int toggle){
  toggle=1-toggle;
  return toggle;
}
