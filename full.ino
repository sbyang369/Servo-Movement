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

#define baseReset 0
#define armReset 135
#define wristReset 90
#define gripperReset 0 

#define baseHarvest 90
#define armHarvest 65
#define wristHarvest 0 
#define gripperHarvest 210


// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(60); 
  Serial.println("PCA9685 initialized.");
}

void loop() {
  reset();
  delay(5000);  // Adjust delay as needed
  harvest();

  //pwm.setPWM(base,0,angleToPulse(90));
  //pwm.setPWM(arm,0,angleToPulse(65));
  //pwm.setPWM(wrist,0,angleToPulse(0));
  //pwm.setPWM(gripper,0,angleToPulse(210));
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: ");
  Serial.print(ang);
  return pulse;
}

void reset() {
  Serial.print("BEGINNING RESET");
  pwm.setPWM(base, 0, angleToPulse(baseReset));
  pwm.setPWM(arm, 0, angleToPulse(armReset));
  pwm.setPWM(wrist, 0, angleToPulse(wristReset));
  pwm.setPWM(gripper, 0, angleToPulse(gripperReset));
  Serial.print("RESET COMPLETE");
}

void harvest() {
  Serial.print("BEGINNING HARVEST");
  pwm.setPWM(base, 0, angleToPulse(baseHarvest));
  pwm.setPWM(arm, 0, angleToPulse(armHarvest));
  pwm.setPWM(wrist, 0, angleToPulse(wristHarvest));
  pwm.setPWM(gripper, 0, angleToPulse(gripperHarvest));
  Serial.print("HARVEST COMPLETE");
}
