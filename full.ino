#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 15
#define I2C_SCL 14
#define SERVOMIN 130
#define SERVOMAX 540

#define base 0 
// #define forearm
#define arm 1
#define wrist 2
#define gripper 3

#define armReset 135
#define wristReset 90
#define gripperReset 0 



// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with specific SDA and SCL pins
  pwm.begin();
  pwm.setPWMFreq(60);  // Set frequency to 60Hz for servos
  Serial.println("PCA9685 initialized.");
}

void loop() {
  delay(5000);
  reset();
  delay(5000);
  harvest();
  delay(5000);
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" -> Pulse: ");
  Serial.println(pulse);
  return pulse;
}

void reset() {
  Serial.println("BEGINNING RESET");
  pwm.setPWM(base, 0, angleToPulse(90));
  delay(500);
  pwm.setPWM(arm, 0, angleToPulse(armReset));
  delay(500);
  pwm.setPWM(wrist, 0, angleToPulse(wristReset));
  delay(500);
  pwm.setPWM(gripper, 0, angleToPulse(gripperReset));
  Serial.println("RESET COMPLETE");
}

void harvest() {
  Serial.println("BEGINNING HARVEST");
  pwm.setPWM(base, 0, angleToPulse(0));
  delay(1000);
  pwm.setPWM(arm, 0, angleToPulse(65));
  delay(1000);
  pwm.setPWM(wrist, 0, angleToPulse(0));
  delay(1000);
  pwm.setPWM(gripper, 0, angleToPulse(210));
  delay(1000);
  Serial.println("HARVEST COMPLETE");
}
