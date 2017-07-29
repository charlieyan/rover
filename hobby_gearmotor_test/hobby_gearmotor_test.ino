#include <SparkFun_TB6612.h>

#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 6

#define BIN1 7
#define BIN2 8
#define PWMB 9

const int offset = 1;
Motor motor_l = Motor(AIN1, AIN2, PWMA, offset, STBY);
Motor motor_r = Motor(BIN1, BIN2, PWMB, offset, STBY);

String readString;

void setup()
{
  Serial.begin(115200);
  Serial.println("starting");
}
 
void loop()
{
  while (Serial.available()) {
    char c = Serial.read(); // gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2); //slow looping to allow buffer to fill with next character
  }
  if (readString.length() > 0) {
    Serial.println(readString);
  }

  if (readString == "start") {
    Serial.println("starting");
    motor_l.drive(255);
    motor_r.drive(255);
  }
  else if (readString == "stop") {
    Serial.println("stopping");
    motor_l.brake();
    motor_r.brake();
  }
  readString = "";
}
