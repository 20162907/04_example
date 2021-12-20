#include <Servo.h>
#include <SharpIR.h>
#include <math.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 252
#define _DIST_TARGETI 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.1
#define AVER 10 //평균값 필터 N

// configurable parameters
#define INTERVAL 20 // sampling interval (unit: ms)
#define _DUTY_MIN 1356
#define _DUTY_NEU 1496 // servo neutral position (90 degree)
#define _DUTY_MAX 1636

// Servo speed control
#define _SERVO_ANGLE 30 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 10 // servo speed limit (deg/sec)
#define _RAMPUP_TIME 360 // servo pped

// Event periods
#define _INTERVAL_DIST 10 // distance sensor interval (ms)
#define _INTERVAL_SERVO 10 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define KP 0.71
#define KI 0.001
#define KD 54.0

#define _ITERM_MAX 200
#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100

// global variables
float timeout; // unit: us
float raw_dist, dist_min, dist_max, dsist_ema, dist_cali2, dist_cali, dist_ema,dist_median; // unit: mm
float dist_final;
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
//int duty_chg_per_interval; // current speed (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
float dist_target;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm, ServoOutput;
SharpIR SharpIR(PIN_IR, 1080);


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = START;
  ir_distance();
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  // initialize serial port
  Serial.begin(2000000);

  // servo related variables
  
  //duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  //duty_chg_adjust = (float) duty_chg_max * INTERVAL / _RAMPUP_TIME;
  //duty_chg_per_interval = 0; // initial speed is set to 0.

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
    iterm += KI * error_curr;

    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    
    control = pterm + dterm + iterm;
    duty_target=_DUTY_NEU+control;
    if(duty_target < _DUTY_MIN) duty_target=_DUTY_MIN;
    if(duty_target > _DUTY_MAX) duty_target=_DUTY_MAX;
    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;
    myservo.writeMicroseconds(duty_target);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_cali);
    Serial.print(",T:");
    Serial.print(_DIST_TARGETI);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(255,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
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
  float a=89;  //150
  float b=195;  //300
  dis = ir_distance();
  dis1 = 150.0 / (b - a) * (dis - a) + 150.0;
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
