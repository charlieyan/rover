// only works off Uno at the moment, needs 5V

#include <Ultrasonic.h>

Ultrasonic ultrasonic(5,6); // (Trig PIN,Echo PIN)
//Ultrasonic ultraright(6,7);  // (Trig PIN,Echo PIN)

void setup() {
  Serial.begin(9600);
  pinMode(4, OUTPUT); // VCC pin
  pinMode(7, OUTPUT); // GND ping
  digitalWrite(4, HIGH); // VCC +5V mode  
  digitalWrite(7, LOW);  // GND mode 
}

void loop()
{
  Serial.print(ultrasonic.Ranging(CM)); // CM or INC
  Serial.print(" cm, " );
  Serial.print(ultrasonic.Timing());
  Serial.println(" ms" ); // milliseconds
  delay(100);
}
