#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define SDA and SCL pins for I2C
#define I2C_SDA 15
#define I2C_SCL 14
#define SERVOMIN 130
#define SERVOMAX 540

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);


void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  if (Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("I2C initialized successfully.");
  } else {
    Serial.println("I2C initialization failed!");
    while (true);
  }

  pwm.begin();
  pwm.setPWMFreq(60); 
  Serial.println("PCA9685 initialized.");
}

void loop() {
  pwm.setPWM(0, 0, angleToPulse(90)); 
  delay(3000);
  pwm.setPWM(0, 0, angleToPulse(180)); 
  delay(3000);

}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}
