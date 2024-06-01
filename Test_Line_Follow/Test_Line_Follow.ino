// define motor driver pin
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENA 5
#define ENB 4
// define encoder pin
#define left_psignal_A 22
#define left_psignal_B 23
#define right_psignal_A 33
#define right_psignal_B 34
// // define motor stepper pin (could be skipped)
// #define step_IN1 32
// #define step_IN2 33
// #define step_IN3 25
// #define step_IN4 26
// define ir analog output pin in esp32
#define ir_tkr 32
#define ir_tgh 33
#define ir_tkn 25
#define ir_kr 26
#define ir_kn 27

//defining motor variable
int speed = 80;
int max_speed = 100, min_speed = 60;
int left_speed, right_speed;

// Define sensor variable
int tkr, tgh, tkn, kr, kn;
int count;

// define pid variable
float Kp = 0.2, Ki = 0, Kd = 0, Ts = 1;
float setpoint = 2047;
float l_last_err = 0, l_delta_err = 0, l_sum_err = 0, l_err = 0;
float r_last_err = 0, r_delta_err = 0, r_sum_err = 0, r_err = 0;

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
  pinMode(15, INPUT);
  Serial.begin(115200);
  digitalWrite(15, LOW);
}

void read_ir(){
  tkr = analogRead(ir_tkr);
  tgh = analogRead(ir_tgh);
  tkn = analogRead(ir_tkn);
  kr = digitalRead(ir_kr);
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
  move_forward(left_speed, right_speed);
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
  move_forward(left_speed, right_speed);
}

void loop() {
  // put your main code here, to run repeatedly:
  read_ir();
  if (tgh >= 2500){
    move_forward(speed, speed);
  }
  else{
    if(kn <= 2500 && kr <= 2500){  
      if(tkr >= 2500 && tkn <= 2500){
        turn_right(speed-10, speed-20);
      }
      else if(tkr <= 2500 && tkn >= 2500){
        turn_left(speed-20, speed-10);
      }
    }
    else{
      if(kn <= 2500 && kr == 1){
        turn_right(speed+20, speed-20);
      }
      else if(kn >= 2500 && kr == 0){
        turn_left(speed-20, speed+20);
      }
    }
  }
    // if (tkr >= 2500 || tkn >= 2500){
    //   if (tkr > tkn){
    //     // test_PID_left();

    //   }
    //   else{
    //     // test_PID_right();
    //   }
    // }
    // else{
    //   move_forward(speed, speed);
    // }
  // if (tgh < 3000){
  //   if (tkr >= 3000 || tkn >= 3000){
  //     if (tkr > tkn){
  //       turn_left(speed-20, speed+20);
  //     }
  //     else{  
  //       turn_right(speed+20, speed-20);
  //     }
  //   }
  //   else {
  //     move_forward(speed-20,speed-20);
  //   }
  // }
}