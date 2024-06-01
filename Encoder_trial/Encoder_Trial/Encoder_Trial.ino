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

// define motor speed
int speed = 102;

// define jarak
float ds, ds1, ds2;

// Define sensor variable
float tkr, tgh, tkn, kr, kn;

// define count variable
int count = 0;
int loop_count = 0;
int line_count = 0;
int i = 1;

// Reading infrared sensor
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
int move_forward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed+2);
}

int move_backward(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed+2);
}

// turn right
int turn_right(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// turn left
int turn_left(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
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

void reset_enc_pos(){
  encoder1Pos = 0;
  encoder2Pos = 0;
}

void reset_ds(){
  ds = 0;
  ds1 = 0;
  ds2 = 0;
}

void rotation(){
  // ppr = 829
  // 20.41 = 829 ppr
}

void task_B(){
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 24 && ds2 < 24){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
    Serial.print(ds1);
    Serial.print(" ");
    Serial.println(ds2);
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 55){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_left();
  while(ds1 < 23 && ds2 < 23){
    ds1 = encoder1Pos * 0.031 * (-1);
    ds2 = encoder2Pos * 0.031;
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 85){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  delay(500);
  for (int j = 0; j < 700; j++){
    // step_motor_up();
    step_motor_down();
  }
  move_backward();
  delay(250);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 54 && ds2 < 54){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
  }
  stop();
  delay(500);
}

void task_B_2(){
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 24 && ds2 < 24){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
    Serial.print(ds1);
    Serial.print(" ");
    Serial.println(ds2);
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 45){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_left();
  while(ds1 < 23 && ds2 < 23){
    ds1 = encoder1Pos * 0.031 * (-1);
    ds2 = encoder2Pos * 0.031;
  }
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 90){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  delay(500);
  for (int j = 0; j < 700; j++){
    // step_motor_up();
    step_motor_down();
  }
  move_backward();
  delay(250);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 48 && ds2 < 48){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
  }
  stop();
  delay(500);
}

void task_C(){
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 82){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  for (int j = 0; j < 700; j++){
    step_motor_up();
    // step_motor_down();
  }
  move_backward();
  delay(500);
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 52 && ds2 < 52){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
  }
  stop();
  delay(500);
}

void task_C_2(){
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  move_forward();
  while (ds < 82){
    ds = ((encoder1Pos + encoder2Pos)/2) * 0.062;
  }
  stop();
  for (int j = 0; j < 700; j++){
    step_motor_up();
    // step_motor_down();
  }
  move_backward();
  delay(500);
  stop();
  delay(500);
  reset_ds();
  reset_enc_pos();
  turn_right();
  while(ds1 < 54 && ds2 < 54){
    ds1 = encoder1Pos * 0.031;
    ds2 = encoder2Pos * 0.031 * (-1);
  }
  stop();
  delay(500);
}

void setup() {
  // Set motor pinMode
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
  // Set encoder pins as inputs
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

void loop(){
  read_ir();
  if (count < 5){
    move_forward();
    if(kr == 1 && tgh == 1 && kn == 1){
      count++;
      delay(300);
    }
  }
  else{
    if(loop_count == 0){
      // task_A();
      task_B();
      loop_count++;
      // task_C();
      count = 0;
    }
    else if (loop_count == 1){
      task_C();
      loop_count++;
      count = 0;
    }
    else if (loop_count == 2){
      task_B_2();
      loop_count++;
      count = 0;
    }
    else if(loop_count == 3){
      task_C_2();
      loop_count++;
      // count = 0;
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