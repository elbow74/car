#include <Arduino.h>

//MOTOR SPEED AND DIRECTION
#define PIN_RSPEED 6 // PWM (ENABLE)
#define PIN_RDIR   7

#define PIN_LSPEED 5 // PWM (ENABLE)
#define PIN_LDIR   4

// HIGH/LOW for PHASE/ENABLE (HIGH) or IN/IN (LOW)
#define PIN_MODE   2

// pin for buzzer
#define PIN_BUZZER 8

// RGB led pins
#define PIN_RED   11
#define PIN_GREEN 10
#define PIN_BLUE  9 


String inputString = "";
bool packetStarted = false;

// VARIABLES: MOTOR CONTROL
int LSPEED = 0;
int LDIR = 0;
int RSPEED = 0;
int RDIR = 0;

// VARIABLES: RGB LED
uint8_t RGB_ON = 0; // 1 is on, 0 off
uint8_t RGB_RED;
uint8_t RGB_GREEN;
uint8_t RGB_BLUE;
uint16_t hue = 0;
unsigned long lastColorUpdate = 0;
const unsigned long rainbowInterval = 50; // ms between color changes

void setup() {
  Serial.begin(115200);

  // PHASE/ENABLE MODE ACTIVATED
  pinMode(PIN_MODE, OUTPUT);
  digitalWrite(PIN_MODE, HIGH);

  // MOTOR PINS
  pinMode(PIN_LSPEED, OUTPUT);
  pinMode(PIN_LDIR, OUTPUT);

  pinMode(PIN_RSPEED, OUTPUT);
  pinMode(PIN_RDIR, OUTPUT);

  // BUZZER PIN
  pinMode(PIN_BUZZER, OUTPUT);

}

void loop() {
  if (RGB_ON) {
    unsigned long now = millis();
    if (now - lastColorUpdate > rainbowInterval) {
      lastColorUpdate = now;
      hue = (hue + 5) % 360; // rotate hue

      hsvToRgb(hue, 255, 128, RGB_RED, RGB_GREEN, RGB_BLUE);
      setRGBColor(RGB_RED, RGB_GREEN, RGB_BLUE);
    }
  }
  else {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE, 0);

  }

  while (Serial.available()>=6) {
    char HEADER_L = Serial.read();
    byte LSPEED_100 = Serial.read();
    byte LDIR   = Serial.read();

    char HEADER_R = Serial.read();
    byte RSPEED_100 = Serial.read();
    byte RDIR   = Serial.read();

    if (HEADER_L == 'L' && HEADER_R == 'R'){ // if packet valid
      if(LSPEED_100 == 1 && RSPEED_100 == 1 && LDIR == 1 && RDIR == 1){
        digitalWrite(PIN_BUZZER, HIGH);
        delay(100);
        digitalWrite(PIN_BUZZER, LOW);
      }
      else if (LSPEED_100 == 1 && LDIR == 1 && RSPEED_100 == 0 && RDIR == 1){
        RGB_ON = 1;
      }
      else if (LSPEED_100 == 1 && LDIR == 1 && RSPEED_100 == 0 && RDIR == 0){
        RGB_ON = 0;
      }
      else {
        int pwmLSPEED = map(LSPEED_100, 0, 100, 0, 200);
        int pwmRSPEED = map(RSPEED_100, 0, 100, 0, 200);

        leftMotor(LDIR, pwmLSPEED);
        rightMotor(RDIR, pwmRSPEED);
      }
    }

    /*
    if (c == '<') {
      inputString = "";           // start fresh
      packetStarted = true;
    } 
    else if (c == '>') {
      packetStarted = false;
      parsePacket(inputString);   // process the collected packet
      
      int pwmLSPEED = map(LSPEED, 0, 100, 0, 255);
      int pwmRSPEED = map(RSPEED, 0, 100, 0, 255);

      leftMotor(LDIR, pwmLSPEED);
      rightMotor(RDIR, pwmRSPEED);
    } 
    else if (packetStarted) {
      inputString += c;           // accumulate characters
    }
    */
  }
}

void setRGBColor(uint8_t R, uint8_t G, uint8_t B){
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}

// controls left motor direction/ state
void leftMotor(int isForward, int speed) {
  digitalWrite(PIN_LDIR, isForward);
  analogWrite(PIN_LSPEED, speed);
}

// controls right motor direction/ state
void rightMotor(int isForward, int speed) {
  digitalWrite(PIN_RDIR, isForward);
  analogWrite(PIN_RSPEED, speed);
}

void hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
  float hh = h / 60.0;
  int i = int(hh);
  float ff = hh - i;
  float p = v * (1.0 - s / 255.0);
  float q = v * (1.0 - (s / 255.0) * ff);
  float t = v * (1.0 - (s / 255.0) * (1.0 - ff));

  switch(i) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }

  r = map(r, 0, 255, 0, 255);
  g = map(g, 0, 255, 0, 255);
  b = map(b, 0, 255, 0, 255);
}

