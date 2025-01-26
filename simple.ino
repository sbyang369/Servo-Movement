#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define I2C_SDA 15
#define I2C_SCL 14
constexpr int SERVOMIN = 130;
constexpr int SERVOMAX = 540;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...")
  pwm.begin();
  pwm.setPWMFreq(60); // Set frequency to 60Hz for servos
  Serial.println("PCA9685 initialized.");
}

// Loop function to test servo movement
void loop() {
  moveServo(0, 0);   // Move to 0 degrees
  delay(1000);
  moveServo(0, 90);  // Move to 90 degrees
  delay(1000);
  moveServo(0, 180); // Move to 180 degrees
  delay(1000);
}

// Function to move a servo to a specified angle
void moveServo(int servo, int angle) {
  int pulse = angleToPulse(angle);
  pwm.setPWM(servo, 0, pulse);
  
  Serial.print("Moving servo ");
  Serial.print(servo);
  Serial.print(" to angle ");
  Serial.print(angle);
}

// Function to convert an angle (0-180) to a pulse width
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}
