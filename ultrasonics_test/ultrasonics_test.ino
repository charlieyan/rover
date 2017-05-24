// only works off Uno at the moment, needs 5V

#include <Ultrasonic.h>

Ultrasonic ultrasonic(14,13); // (Trig PIN,Echo PIN), 5V tolerant pins
//Ultrasonic ultraright(6,7);  // (Trig PIN,Echo PIN)

void setup() {
  Serial.begin(9600);
  // Make sure VCC is V_IN 5V on teensy 3.2
}

void loop()
{
  Serial.print(ultrasonic.Ranging(CM)); // CM or INC
  Serial.print(" cm, " );
  Serial.print(ultrasonic.Timing());
  Serial.println(" ms" ); // milliseconds
  delay(100);
}
