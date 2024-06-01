// define motor driver pin
#define IN1 4
#define IN2 7
#define IN3 2
#define IN4 3
#define ENA 6
#define ENB 5
// define ir pin
#define ir_kr A4
#define ir_tkr A3
#define ir_tgh A2
#define ir_tkn A1
#define ir_kn A0
// define motor stepper pin
#define step_IN1 8
#define step_IN2 9
#define step_IN3 10
#define step_IN4 11

// Define sensor variable
float tkr, tgh, tkn, kr, kn;
int count = 0;

//defining motor variable
int speed = 100;
int max_speed = 150, min_speed = 50;
int left_speed, right_speed;

// define pid variable
float Kp = 13, Ki = 0, Kd = 0, Ts = 1;
float last_err = 0, delta_err = 0, sum_err = 0, err = 0;

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

int error(){
  if (kn == 1){
    err = 2;
  }
  else if (kr == 1){
    err = -2;
  }
  else if (tkn == 1){
    err = 1;
  }
  else if (tkr == 1){
    err = -1;
  }
  else if (tgh == 1){
    err = 0;
  }
  else {
    err = last_err;
  }
  return err;
}

void test_PID_left(){
  err = error();
  delta_err = err - last_err;
  sum_err += last_err;
  float P = Kp * err;
  float I = Ki * sum_err * Ts ;
  float D = ((Kd / Ts) * delta_err);
  last_err = err;
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
  if (err == -2){
    turn_left(left_speed, right_speed);
  }
  else if (err == 2){
    turn_right(left_speed, right_speed);
  }
  else{
    move_forward(left_speed, right_speed);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(step_IN1, OUTPUT);
  pinMode(step_IN2, OUTPUT);
  pinMode(step_IN3, OUTPUT);
  pinMode(step_IN4, OUTPUT);
  pinMode(ir_tkr, INPUT);
  pinMode(ir_tgh, INPUT);
  pinMode(ir_tkn, INPUT);
  pinMode(ir_kr, INPUT);
  pinMode(ir_kn, INPUT);
  Serial.begin(9600);  
}

void loop() {
  read_ir();  //read infrared sensor
  // count horizontal line until 5
  if (count < 5){
    move_forward(speed, speed);
    if(kr == 1 && tgh == 1 && kn == 1){
      count++;
      delay(100);
    }
  }
  // do pid controller
  else{
    test_PID_left();
  }
}