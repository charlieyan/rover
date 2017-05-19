#include <SparkFun_TB6612.h>

#define PWMB 23
#define BIN2 22
#define BIN1 21
#define STBY 20

const int offsetB = 1;
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

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
    motor2.drive(255,1000);
    motor2.drive(-255,1000);
    motor2.brake();
    delay(1000);
  }
  else if (readString == "stop") {

  }
  readString = "";

  motor2.drive(255,1000);
  motor2.drive(-255,1000);
//  motor2.brake();
  Serial.println("drove it");
}
