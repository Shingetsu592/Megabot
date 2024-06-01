// define motor driver pin
#define IN1 22
#define IN2 23
#define IN3 25
#define IN4 24
#define ENA 6
#define ENB 5
// define Encoder Pin
#define ENC1A 2 //kiri
#define ENC1B 4 //kiri
#define ENC2A 3 //kanan
#define ENC2B 7 //kanan
// define ir pin
#define ir_kr A3
#define ir_tkr A2
#define ir_tgh A4
#define ir_tkn A1
#define ir_kn A0
// define motor stepper pin
#define step_IN1 8
#define step_IN2 9
#define step_IN3 10
#define step_IN4 11

// define encoder variable
volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

// Define sensor variable
float tkr, tgh, tkn, kr, kn;

// define counting variable
int mid_count = 0;
int left_count = 0;
int right_count = 0;
int start_count = 0;
int switch_count = 0;

// define distance variable
int ds = 0;


//defining motor variable
int speed = 102;
int max_speed = 152, min_speed = 52;
int left_speed, right_speed;

// define pid variable
float Kp = 15, Ki = 0, Kd = 0, Ts = 1;
float last_err = 0, delta_err = 0, sum_err = 0, err = 0;
// float last_err = 0, delta_err = 0, sum_err = 0, err = 0;

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

void reset_enc_pos(){
  encoder1Pos = 0;
  encoder2Pos = 0;
}

void reset_ds(){
  ds = 0;
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

// move backward
int move_backward(int l_speed, int r_speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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

int error1(){
  /*
    prioritize turn right
  */
  if (kn == 1 && tkn == 1){
    err = 3;
  }
  else if (kr == 1 && tkr == 1){
    err = -3;
  }
  else if (kn == 1){
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

int error2(){
  /*
    prioritize turn left
  */
  if (kr == 1 && tkr == 1){
    err = -3;
  }
  else if (kn == 1 && tkn == 1){
    err = 3;
  }
  else if (kr == 1){
    err = -2;
  }
  else if (kn == 1){
    err = 2;
  }
  else if (tkr == 1){
    err = -1;
  }
  else if (tkn == 1){
    err = 1;
  }
  else if (tgh == 1){
    err = 0;
  }
  else {
    err = last_err;
  }
  return err;
}

void test_PID_left(int errs){
  err = errs;
  delta_err = err - last_err;
  sum_err += last_err;
  float P = Kp * err;
  float I = Ki * sum_err * Ts ;
  float D = ((Kd / Ts) * delta_err);
  last_err = err;
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

/*
  if left more than right
    prioritize right turn and count left (positive err value)
    use error1
  else
    prioritize left turn and count right (negative err value)
    use error2
*/

void mission_A(){
  /* 
    total mid count = 4
    total left count = 6
    total right count = 5
  */
  if (kr == 1 && tgh == 1 && kn == 1){
    if(mid_count == 4){
      turn_right(left_speed, right_speed);
      delay(300);
    }
    else{
      move_forward(left_speed, right_speed);
      delay(50);
      mid_count++;
    }
  }
  else if(err == 3 && mid_count == 0){
    turn_right(left_speed, right_speed);
    delay(300);
    mid_count++;
  }
  else if (err == -3){
    if(left_count < 6){
      move_forward(left_speed, right_speed);
      if (err == -3){
        left_count++;
      }
    }
    else if (left_count == 6){
      turn_left(left_speed, right_speed);
      delay(300);
    }
  }
}

void mission_B(){
  /* 
    total mid count = 4
    total left count = 5
    total right count = 6
  */
  if (kr == 1 && tgh == 1 && kn == 1){
    if(mid_count == 4){
      turn_left(left_speed, right_speed);
      delay(300);
    }
    else{
      move_forward(left_speed, right_speed);
      // delay(50);
      mid_count++;
    }
  }
  else if(err == -3 && mid_count == 0){
    turn_left(left_speed, right_speed);
    delay(300);
    mid_count++;
  }
  else if (err == -3){
    if(right_count < 5){
      move_forward(left_speed, right_speed);
      if (err == -3){
        right_count++;
      }
    }
    else if (right_count == 5){
      turn_right(left_speed, right_speed);
      delay(300);
    }
  }
}

void manipulator_up(){
  reset_enc_pos();
  reset_ds();
  move_forward(left_speed, right_speed);
  while(ds < 7.5){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  // pick object
  move_backward(speed, speed);
  delay(250);
  while(tgh != 1){
    turn_left(speed, speed);
  }
}

void manipulator_down(){
  reset_enc_pos();
  reset_ds();
  move_forward(left_speed, right_speed);
  while(ds < 7.5){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  // drop object
  move_backward(speed, speed);
  delay(250);
  while(tgh != 1){
    turn_right(speed, speed);
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
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  pinMode(ENC2A, INPUT_PULLUP); 
  pinMode(ENC2B, INPUT_PULLUP);
  Serial.begin(9600);
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
}

void loop() {
  read_ir();  //read infrared sensor
  if (start_count == 0){
    move_forward(speed, speed);
    if (kr == 1 && tgh == 1 && kn == 1){
      start_count++;
      turn_right(speed, speed);
      delay(100);
    }
  }
  else{
    if(switch_count%2 == 0){
      test_PID_left(error1());
      mission_A();
      switch_count++;
      manipulator_up();
    }
    else if(switch_count%2 == 1){
      test_PID_left(error2());
      mission_B();
      switch_count++;
      manipulator_down();
    }
  }
}

// Incremental encoder logic
void readEncoder1(){
  if(digitalRead(ENC1A) == digitalRead(ENC1B)) {
    encoder1Pos--; // CCW or mundur
  } else {
    encoder1Pos++; //CW or maju
  }
}

void readEncoder2(){
  if(digitalRead(ENC2A) == digitalRead(ENC2B)) {
    encoder2Pos++; //CW or maju
  } else {
    encoder2Pos--; // CCW or mundur
  }  
}