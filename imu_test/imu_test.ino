#include <SparkFun_ADXL345.h>

ADXL345 adxl = ADXL345(); // remember, teensy 3.0-3.6 require pull-up resistors on I2C pins, 4.7k

void setup() {
  Serial.begin(9600);
  Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  Serial.println();
  adxl.powerOn();
  adxl.setRangeSetting(16); // Give the range settings
}

int x,y,z;
void loop() {
  adxl.readAccel(&x, &y, &z);
  
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
}

// calibration

//int AccelMinX = 0;
//int AccelMaxX = 0;
//int AccelMinY = 0;
//int AccelMaxY = 0;
//int AccelMinZ = 0;
//int AccelMaxZ = 0; 
//
//int accX = 0;
//int accY = 0;
//int accZ = 0;
//
///************** DEFINED VARIABLES **************/
///*                                             */
//#define offsetX   -123       // OFFSET values
//#define offsetY   -16
//#define offsetZ   -10
//
//#define gainX     133        // GAIN factors
//#define gainY     261
//#define gainZ     248 
//
///******************** SETUP ********************/
///*          Configure ADXL345 Settings         */
//void setup()
//{
//  Serial.begin(9600);                 // Start the serial terminal
//  Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
//  Serial.println();
//  
//  adxl.powerOn();                     // Power on the ADXL345
//
//  adxl.setRangeSetting(2);           // Give the range settings
//                                      // Accepted values are 2g, 4g, 8g or 16g
//                                      // Higher Values = Wider Measurement Range
//                                      // Lower Values = Greater Sensitivity
//                                      
//  adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
//                                      // Default: Set to 1
//                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
//}
//
///****************** MAIN CODE ******************/
///*  Accelerometer Readings and Min/Max Values  */
//void loop()
//{
//  Serial.println("Send any character to display values.");
//  while (!Serial.available()){}       // Waiting for character to be sent to Serial
//  Serial.println();
//  
//  // Get the Accelerometer Readings
//  int x,y,z;                          // init variables hold results
//  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store in variables x,y,z
//
//  if(x < AccelMinX) AccelMinX = x;
//  if(x > AccelMaxX) AccelMaxX = x;
//
//  if(y < AccelMinY) AccelMinY = y;
//  if(y > AccelMaxY) AccelMaxY = y;
//
//  if(z < AccelMinZ) AccelMinZ = z;
//  if(z > AccelMaxZ) AccelMaxZ = z;
//
//  Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
//  Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
//  Serial.println();
//
//  
//  /* Note: Must perform offset and gain calculations prior to seeing updated results
//  /  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
//  /  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
//  /  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */
//
//  // UNCOMMENT SECTION TO VIEW NEW VALUES
//  //accX = (x - offsetX)/gainX;         // Calculating New Values for X, Y and Z
//  //accY = (y - offsetY)/gainY;
//  //accZ = (z - offsetZ)/gainZ;
//
//  //Serial.print("New Calibrated Values: "); Serial.print(accX); Serial.print("  "); Serial.print(accY); Serial.print("  "); Serial.print(accZ);
//  //Serial.println(); 
//  
//  while (Serial.available())
//  {
//    Serial.read();                    // Clear buffer
//  }
//}

// sample

//void setup(){
//  
//  Serial.begin(9600);                 // Start the serial terminal
//  Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
//  Serial.println();
//  adxl.powerOn();                     // Power on the ADXL345
//  adxl.setRangeSetting(16);           // Give the range settings
//                                      // Accepted values are 2g, 4g, 8g or 16g
//                                      // Higher Values = Wider Measurement Range
//                                      // Lower Values = Greater Sensitivity
//
//  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
//  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
// 
//  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
//  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
//  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?
//
//  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
//  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
//  adxl.setTapThreshold(50);           // 62.5 mg per increment
//  adxl.setTapDuration(15);            // 625 μs per increment
//  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
//  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
// 
//  // Set values for what is considered FREE FALL (0-255)
//  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
//  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
// 
//  // Setting all interupts to take place on INT1 pin
//  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
//                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
//                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
//  
//  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
//  adxl.InactivityINT(1);
//  adxl.ActivityINT(1);
//  adxl.FreeFallINT(1);
//  adxl.doubleTapINT(1);
//  adxl.singleTapINT(1);
//}
//
///****************** MAIN CODE ******************/
///*     Accelerometer Readings and Interrupt    */
//void loop(){
//  int x,y,z;   
//  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
//  ADXL_ISR();
//
////  Serial.print(x);
////  Serial.print(", ");
////  Serial.print(y);
////  Serial.print(", ");
////  Serial.println(z); 
//}
//
///********************* ISR *********************/
///* Look for Interrupts and Triggered Action    */
//void ADXL_ISR() {
//  // getInterruptSource clears all triggered actions after returning value
//  // Do not call again until you need to recheck for triggered actions
//  byte interrupts = adxl.getInterruptSource();
//  
//  // Free Fall Detection
//  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
//    Serial.println("*** FREE FALL ***");
//    //add code here to do when free fall is sensed
//  } 
//  
//  // Inactivity
//  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
//    Serial.println("*** INACTIVITY ***");
//     //add code here to do when inactivity is sensed
//  }
//  
//  // Activity
//  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
//    Serial.println("*** ACTIVITY ***"); 
//     //add code here to do when activity is sensed
//  }
//  
//  // Double Tap Detection
//  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
//    Serial.println("*** DOUBLE TAP ***");
//     //add code here to do when a 2X tap is sensed
//  }
//  
//  // Tap Detection
//  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
//    Serial.println("*** TAP ***");
//     //add code here to do when a tap is sensed
//  } 
//}

