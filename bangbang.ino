
#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13
#define PIN_IR A0
#define PIN_LED 9

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1436 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)
#define _DIST_ALPHA 0.05

#define MEDIAN_N 15  //중위수 필터 N

// global variables
int a, b; // unit: mm
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, dist_final; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float median_arr[MEDIAN_N],median,median_sum; //필터 저장할 배열
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

  a =79; //70;
  b = 270; //300;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() { //raw-cali-ema-median

  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_ema = _DIST_ALPHA*dist_cali+(1-_DIST_ALPHA)*dist_ema;
  for (int i=(MEDIAN_N-1);i>=1;i--){ //median[]배열을 한칸씩 미룸, [0]에 raw를 넣기 위해,오른쪽에서부터
     median_arr[i]=median_arr[i-1];
  }
  median_arr[0]=dist_ema;
  median_sum=0;
  for (int i=0;i<=(MEDIAN_N-1);i++){ 
     median_sum+=median_arr[i];
  }
  dist_final = median_sum/MEDIAN_N;
  Serial.print("max:500");
  Serial.print(",");
  Serial.print("dist_ema:");
  Serial.print(dist_ema);
  Serial.print(",");
  Serial.print(" dist_final:");
  Serial.println(dist_final);
//  myservo.writeMicroseconds(_DUTY_NEU-650);
  if(dist_final > 255) {
    myservo.writeMicroseconds(_DUTY_NEU-40);
  }
  else{
    myservo.writeMicroseconds(_DUTY_NEU+50);
  }

// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
