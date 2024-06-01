#include <HardwareSerial.h>

// define encoder pin
#define left_psignal_A 18
#define left_psignal_B 19
#define right_psignal_A 16
#define right_psignal_B 17
// define motor stepper pin (could be skipped)
#define step_IN1 26
#define step_IN2 27
#define step_IN3 14
#define step_IN4 12
// define ultrasonic pin
// #define echo_pin
// #define trig_pin
// define ir pin
#define ir_kr 25
#define ir_tkr 33
#define ir_tgh 32
#define ir_tkn 35
#define ir_kn 34

HardwareSerial Arduino(2);

// Define sensor variable
float tkr, tgh, tkn, kr, kn;
int count, cs = 4;

//defining motor variable
int speed = 80;
int max_speed = 100, min_speed = 60;
int left_speed, right_speed;

// define pid variable
float Kp = 0.2, Ki = 0, Kd = 0, Ts = 1;
float setpoint = 2047;
float l_last_err = 0, l_delta_err = 0, l_sum_err = 0, l_err = 0;
float r_last_err = 0, r_delta_err = 0, r_sum_err = 0, r_err = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(step_IN1, OUTPUT);
  pinMode(step_IN2, OUTPUT);
  pinMode(step_IN3, OUTPUT);
  pinMode(step_IN4, OUTPUT);
  pinMode(ir_tkr, INPUT);
  pinMode(ir_tgh, INPUT);
  pinMode(ir_tkn, INPUT);
  pinMode(ir_kr, INPUT);
  pinMode(ir_kn, INPUT);
  Serial.begin(115200);
}

void read_ir(){
  tkr = digitalRead(ir_tkr);
  tgh = digitalRead(ir_tgh);
  tkn = digitalRead(ir_tkn);
  kr = digitalRead(ir_kr);
  kn = digitalRead(ir_kn);
  Serial.print(kr);
  Serial.print(" ");
  Serial.print(tkr);
  Serial.print(" ");
  Serial.print(tgh);
  Serial.print(" ");
  Serial.print(tkn);
  Serial.print(" ");
  Serial.println(kn);
}

void step_motor_up(){
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, HIGH);
  delay(5);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, HIGH);
  digitalWrite(step_IN4, LOW);
  delay(5);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, HIGH);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(5);
  digitalWrite(step_IN1, HIGH);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(5);
}

void step_motor_down(){
  // Rotate CW slowly at 5 RPM
  digitalWrite(step_IN1, HIGH);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(5);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, HIGH);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(5);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, HIGH);
  digitalWrite(step_IN4, LOW);
  delay(5);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, HIGH);
  delay(5);
}

void test_PID_left(){
  l_err = setpoint - tkr;
  l_delta_err = l_err - l_last_err;
  l_sum_err += l_last_err;
  float P = Kp * l_err;
  float I = Ki * l_sum_err * Ts;
  float D = ((Kd / Ts) * l_delta_err);
  l_last_err = l_err;
  float PID = P + I + D;
  left_speed = speed + PID;     // Motor Kiri
  right_speed = speed - PID;     // Motor Kanan
  if(left_speed < min_speed){
    left_speed = min_speed;
  }
  else if(left_speed > max_speed){
    left_speed = max_speed;
  }
  if(right_speed < min_speed){
    right_speed = min_speed;
  }
  else if(right_speed > max_speed){
    right_speed = max_speed;
  }
}

void test_PID_right(){
  r_err = setpoint - tkn;
  r_delta_err = r_err - r_last_err;
  r_sum_err += r_last_err;
  float P = Kp * r_err;
  float I = Ki * r_sum_err * Ts;
  float D = ((Kd / Ts) * r_delta_err);
  r_last_err = r_err;
  float PID = P + I + D;
  left_speed = speed - PID;     // Motor Kiri
  right_speed = speed + PID;     // Motor Kanan
  if(left_speed < min_speed){
    left_speed = min_speed;
  }
  else if(left_speed > max_speed){
    left_speed = max_speed;
  }
  if(right_speed < min_speed){
    right_speed = min_speed;
  }
  else if(right_speed > max_speed){
    right_speed = max_speed;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  read_ir();
  if(tgh == 0){
    if(kn == 1){
      cs = 0;
    }
    else if (kr == 1){
      cs = 1;
    }
    else if(tkn == 1){
      cs = 2;
    }
    else if(tkr == 1){
      cs = 3;
    }
  }
  else{
    if(kn == 1){
      cs = 0;
    }
    else if (kr == 1){
      cs = 1;
    }
    else{
      cs = 4;
    }
  }
  Serial.println();
  Serial.println(cs);
  Arduino.print(cs);
  // Serial.write(cs);
}
