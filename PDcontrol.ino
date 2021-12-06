#include <Servo.h>
#include <SharpIR.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.1
#define AVER 10  //중위수 필터 N

// configurable parameters
#define INTERVAL 20 // sampling interval (unit: ms)
#define _DUTY_MIN 1406
#define _DUTY_NEU 1506 // servo neutral position (90 degree)
#define _DUTY_MAX 1636

// Servo speed control
#define _SERVO_ANGLE 30 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 30 // servo speed limit (deg/sec)

// Event periods
#define _INTERVAL_DIST 20 // distance sensor interval (ms)
#define _INTERVAL_SERVO 20 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define KP 0.7
#define KI 0.0
#define KD 59.00

SharpIR SharpIR(PIN_IR, 1080);

// global variables
int a, b; // unit: mm
float timeout; // unit: us
float raw_dist, dist_min, dist_max, dsist_ema, dist_cali2, dist_cali, dist_ema,dist_median; // unit: mm
float dist_final;
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;
float dist_target;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
         last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm, ServoOutput;




void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  myservo.attach(PIN_SERVO);
  dist_ema=ir_distance();
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(2000000);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;

  // initialize last sampling time
  last_sampling_time = last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
}






void loop() {

  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL) return;
  unsigned long time_curr = millis();


  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    event_dist = false;
    dist_cali = ir_distance_filtered();
    
    error_curr = _DIST_TARGET - dist_cali;
    pterm = KP * error_curr;
    dterm = KD * (error_curr - error_prev);
    control = pterm + dterm;
    duty_target=_DUTY_NEU+control;
    if(duty_target < _DUTY_MIN) duty_target=_DUTY_MIN;
    if(duty_target > _DUTY_MAX) duty_target=_DUTY_MAX;
    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;
    myservo.writeMicroseconds(duty_target);
    //myservo.writeMicroseconds(_DUTY_NEU);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_cali);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }

  // update last sampling time
  last_sampling_time += INTERVAL;
}


float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return SharpIR.distance() * 10;
}

float ir_distance_filtered(void) {
  float dis, dis1, dis2, ema, arr[AVER], sum;
  dis = ir_distance();
  dis1 = 150 + 300.0 / (265 - 87) * (dis - 87);
  dist_ema = _DIST_ALPHA*dis1+(1-_DIST_ALPHA)*dist_ema;
  sum=0;
  for(int i=AVER-2;i>=0;i--){
    arr[i+1]=arr[i];
  }
  arr[0]=dist_ema;
  for(int i=0;i<AVER;i++){
    sum+=arr[i];
  }
  return sum/AVER;
}
