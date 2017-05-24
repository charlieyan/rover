#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

//    int eeAddress = 0;
//    long bnoID;
//    bool foundCalib = false;
//
//    EEPROM.get(eeAddress, bnoID);
//
//    adafruit_bno055_offsets_t calibrationData;
//    sensor_t sensor;
//
//    /*
//    *  Look for the sensor's unique ID at the beginning oF EEPROM.
//    *  This isn't foolproof, but it's better than nothing.
//    */
//    bno.getSensor(&sensor);
//    if (bnoID != sensor.sensor_id)
//    {
//        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
//        delay(500);
//    }
//    else
//    {
//        Serial.println("\nFound Calibration for this sensor in EEPROM.");
//        eeAddress += sizeof(long);
//        EEPROM.get(eeAddress, calibrationData);
//
//        displaySensorOffsets(calibrationData);
//
//        Serial.println("\n\nRestoring Calibration data to the BNO055...");
//        bno.setSensorOffsets(calibrationData);
//
//        Serial.println("\n\nCalibration data loaded into BNO055");
//        foundCalib = true;
//    }
//
//    delay(1000);
//
//    /* Display some basic information on this sensor */
//    displaySensorDetails();
//
//    /* Optional: Display current status */
//    displaySensorStatus();
//
//    bno.setExtCrystalUse(true);
//
//    sensors_event_t event;
//    bno.getEvent(&event);
//    if (foundCalib){
//        Serial.println("Move sensor slightly to calibrate magnetometers");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    else
//    {
//        Serial.println("Please Calibrate Sensor: ");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//
//            Serial.print("X: ");
//            Serial.print(event.orientation.x, 4);
//            Serial.print("\tY: ");
//            Serial.print(event.orientation.y, 4);
//            Serial.print("\tZ: ");
//            Serial.print(event.orientation.z, 4);
//
//            /* Optional: Display calibration status */
//            displayCalStatus();
//
//            /* New line for the next sample */
//            Serial.println("");
//
//            /* Wait the specified delay before requesting new data */
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//
//    Serial.println("\nFully calibrated!");
//    Serial.println("--------------------------------");
//    Serial.println("Calibration Results: ");
//    adafruit_bno055_offsets_t newCalib;
//    bno.getSensorOffsets(newCalib);
//    displaySensorOffsets(newCalib);
//
//    Serial.println("\n\nStoring calibration data to EEPROM...");
//
//    eeAddress = 0;
//    bno.getSensor(&sensor);
//    bnoID = sensor.sensor_id;
//
//    EEPROM.put(eeAddress, bnoID);
//
//    eeAddress += sizeof(long);
//    EEPROM.put(eeAddress, newCalib);
//    Serial.println("Data stored to EEPROM.");
//
//    Serial.println("\n--------------------------------\n");
//    delay(500);

//  /* Display some basic information on this sensor */
//  displaySensorDetails();
//
//  /* Optional: Display current status */
//  displaySensorStatus();
//
//  bno.setExtCrystalUse(true);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);

//    /* Get a new sensor event */
//    sensors_event_t event;
//    bno.getEvent(&event);
//
//    /* Display the floating point data */
//    Serial.print("X: ");
//    Serial.print(event.orientation.x, 4);
//    Serial.print("\tY: ");
//    Serial.print(event.orientation.y, 4);
//    Serial.print("\tZ: ");
//    Serial.print(event.orientation.z, 4);
//
//    /* Optional: Display calibration status */
//    displayCalStatus();
//
//    /* Optional: Display sensor status (debug only) */
//    //displaySensorStatus();
//
//    /* New line for the next sample */
//    Serial.println("");
//
//    /* Wait the specified delay before requesting new data */
//    delay(BNO055_SAMPLERATE_DELAY_MS);
//
//  /* Get a new sensor event */
//  sensors_event_t event;
//  bno.getEvent(&event);
//
//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4);
//
//  /* Optional: Display calibration status */
//  displayCalStatus();
//
//  /* Optional: Display sensor status (debug only) */
//  //displaySensorStatus();
//
//  /* New line for the next sample */
//  Serial.println("");
//
//  /* Wait the specified delay before requesting nex data */
//  delay(BNO055_SAMPLERATE_DELAY_MS);
}
