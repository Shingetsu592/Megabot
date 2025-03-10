// define library
#include <AccelStepper.h>
#include <line_follow.h>

// define motor driver pin
#define IN1 19
#define IN2 18
#define IN3 17
#define IN4 16
#define ENA 5
#define ENB 4
// define encoder pin
#define left_psignal_A 22
#define left_psignal_B 23
#define right_psignal_A 33
#define right_psignal_B 34
// define motor stepper pin (could be skipped)
#define step_IN1 32
#define step_IN2 33
#define step_IN3 25
#define step_IN4 26
// define ir analog output pin in esp32
#define ir_tkr 14
#define ir_tgh 12
#define ir_tkn 13
#define ir_kr 15
#define ir_kn 27

//defining motor variable
int speed = 80;
int max_speed = 100, min_speed = 60;
int speedL, speedR;

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, step_IN1, step_IN3, step_IN2, step_IN4);

// Define sensor variable
int tkr, tgh, tkn, kr, kn;
int count;

Line_follow bot= Line_follow(tkr, tgh, tkn);

// define pid variable
int set_point = 2047;
double Kp = 0.2;       // Proporsional 
double Ki = 0;      // Integral     
double Kd = 0;      // Diferensial
double Ts = 1;        // Time sampling

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(left_psignal_A, INPUT);
  pinMode(left_psignal_B, INPUT);
  pinMode(right_psignal_A, INPUT);
  pinMode(right_psignal_B, INPUT);
  pinMode(ir_tkr, INPUT);
  pinMode(ir_tgh, INPUT);
  pinMode(ir_tkn, INPUT);
  pinMode(ir_kr, INPUT);
  pinMode(ir_kn, INPUT);
  Serial.begin(115200);
  // Set the maximum steps per second:
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(100);
  stepper.setSpeed(300);
}

void read_ir(){
  tkr = analogRead(ir_tkr);
  tgh = analogRead(ir_tgh);
  tkn = analogRead(ir_tkn);
  kr = analogRead(ir_kr);
  kn = analogRead(ir_kn);
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

int move_forward(int l_speed, int r_speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

int turn_right(int l_speed, int r_speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

int turn_left(int l_speed, int r_speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, l_speed);
  analogWrite(ENB, r_speed);
}

void test_stepper(){
    stepper.moveTo(2038);
    stepper.run();
    // delay(10000);
    // stepper.moveTo(-2038);
    // stepper.run();
}

void line_detected(){
  if (kr < 3000 && kn > 3000){
    turn_right(speed-30, speed-30);      // left side
    // turn_left(speed-30, speed-30);     // right side
    delay(100);
  }
  else{
    turn_left(speed-30, speed-30);      // left side
    // turn_right(speed-30, speed-30);     // right side
    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  read_ir();
  if (tgh < 3000){
    if (count < 5){
      move_forward(speed, speed);
    }
    else if (count == 5){
      line_detected();
    }
  }
  else if(tgh >= 3000){
    if (count < 5){
      move_forward(speed, speed);
      count++;
    }
    else if(count == 5){
      bot.pid(set_point, Ki, Kp, Kd, Ts);
      bot.follow(speed, max_speed, min_speed);
      // line_follow();
    }
  }
  
  
  // move_forward(speed, speed);
  // test_stepper();
}
