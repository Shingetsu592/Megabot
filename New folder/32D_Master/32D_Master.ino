#include <SoftwareSerial.h>

// define motor driver pin
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENA 5
#define ENB 6

SoftwareSerial ESP32 (9, 8); // 9 rx, 8 tx

// // setting PWM properties
// const int freq = 5000;
// const int ledChannelA = 7;
// const int ledChannelB = 9;
// const int resolution = 4;

int speed = 100;
int max_speed = 120, min_speed = 80;
int left_speed, right_speed;

int cs;

// HardwareSerial Serial32U(2);

void setup() {
  // put your setup code here, to run once:
  // Serial32U.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
  ESP32.begin(9600);
  // configure LED PWM functionalitites
  // ledcSetup(ledChannelA, freq, resolution);
  // ledcSetup(ledChannelB, freq, resolution);
  // // attach the channel to the GPIO to be controlled
  // ledcAttachPin(ENA, ledChannelA);
  // ledcAttachPin(ENA, ledChannelB);
}

// motor movement
// move forward
int move_forward(int l_speed, int r_speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

// turn right
int turn_right(int l_speed, int r_speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

// turn left
int turn_left(int l_speed, int r_speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

int stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  cs = Serial.read();
  // Serial.println(cs);
  // if(cs == 0){
  //   turn_right(speed+10, speed);
  // }
  // else if(cs == 1){
  //   turn_left(speed, speed+10);
  // }
  // else if(cs == 2){
  //   move_forward(speed, speed+20);
  // }
  // else if(cs == 3){
  //   move_forward(speed+20, speed);
  // }
  // else if(cs == 4){
  //   move_forward(speed, speed);
  // }
  move_forward(speed,speed);
}