 #include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 15
#define I2C_SCL 14
#define SERVOMIN 130
#define SERVOMAX 540

#define base 0 
#define forearm 1
#define arm 2
#define wrist 3
#define gripper 4
#define forearm 5

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
  //pwm.setPWM(arm,0,angleToPulse(360));
  delay(5000);
  reset();
  delay(5000);
  harvest();
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 375, SERVOMIN, SERVOMAX);
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" -> Pulse: ");
  Serial.println(pulse);
  return pulse;
}

void reset() {
  Serial.println("BEGINNING RESET");
  pwm.setPWM(wrist,0,angleToPulse(0)); //YES
  delay(500);

  pwm.setPWM(arm, 0, angleToPulse(180)); //YES
  delay(500);

  pwm.setPWM(base, 0, angleToPulse(220)); //YES
  delay(500);

 // pwm.setPWM(gripper, 0, angleToPulse(0));
 // delay(500);

  Serial.println("RESET COMPLETE");
}

void harvest() {
  Serial.println("BEGINNING HARVEST");

  pwm.setPWM(wrist,0,angleToPulse(0)); //YES
  delay(500);

  pwm.setPWM(arm, 0, angleToPulse(0)); //YES
  delay(500);

  pwm.setPWM(base, 0, angleToPulse(0)); //YES
  delay(500);

 // pwm.setPWM(gripper, 0, angleToPulse(210));
 // delay(500);

  Serial.println("HARVEST COMPLETE");
}

void stopPWM() {
  Serial.println("Stopping all PWM signals...");
  for (int i = 0; i < 16; i++) {  // Assuming 16 channels
    pwm.setPWM(i, 0, 0);
  }
}

void moveServoSmoothly(int channel, int startAngle, int endAngle) {
  int step = (startAngle > endAngle) ? -5 : 5;  // Determine direction (decreasing or increasing)

  for (int angle = startAngle; angle != endAngle + step; angle += step) {
    Serial.print("Moving arm to: ");
    Serial.println(angle);
    pwm.setPWM(channel, 0, angleToPulse(angle));
    delay(500);
  }
}

