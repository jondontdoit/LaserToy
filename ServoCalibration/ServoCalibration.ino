#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// our servo # counter
int pulse = 290;
int pulseLast = 0;
uint8_t servonum = 0;

void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(1000);
  pwm.setPWM(0, 0, 295);
  pwm.setPWM(1, 0, 205);

  Serial.println("Ready!");
}

void loop() {

  if (Serial.available()) {
    byte r = Serial.read();
    if (r == '[') pulse -= 10;
    if (r == ']') pulse += 10;
  }
  
  if (pulse != pulseLast) {
    Serial.println(pulse);
    pwm.setPWM(servonum, 0, pulse);
    delay(100);
    pulseLast = pulse;
  }

}
