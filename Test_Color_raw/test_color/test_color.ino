#include <Adafruit_TCS34725.h>
// define color sensor pin
#define LED 13
#define SDA A4
#define SCL A5

//color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float red, green, blue;

void read_color(){
  tcs.setInterrupt(false);  // turn on LED
  delay(100);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  Serial.print(red);
  Serial.print(" ");
  Serial.print(green);
  Serial.print(" ");
  Serial.println(blue);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // read_color();
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("Red: "); Serial.print(r); Serial.print(" ");
  Serial.print("Green: "); Serial.print(g); Serial.print(" ");
  Serial.print("Blue: "); Serial.print(b); Serial.print(" ");
  Serial.print("Color: "); Serial.print(c); Serial.print(" ");
  Serial.println(" ");
  analogWrite (LED, 255); 
  delay(100);
}
