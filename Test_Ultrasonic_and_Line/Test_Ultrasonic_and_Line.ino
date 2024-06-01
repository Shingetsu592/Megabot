#define trig_ultra 27
#define echo_ultra 12
#define line1 32
#define line2 33

float ultra, dline, duration, aline;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(trig_ultra, OUTPUT);
    pinMode(echo_ultra, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  aline = analogRead(line1);
  dline = digitalRead(line2);
  // Clears the trigPin
  digitalWrite(trig_ultra, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig_ultra, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_ultra, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_ultra, HIGH, 20000);
  // Calculating the distance
  ultra = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(ultra);
  Serial.print("digital line detection: ");
  Serial.println(dline);
  Serial.print("analog line detection: ");
  Serial.println(aline);
  delay(100);
}

