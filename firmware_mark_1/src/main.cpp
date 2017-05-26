// basics
#include <Arduino.h>

// motors, pins 9, 8, 7, 6, 5, 4, 3 (PWM), left of Teensy mount
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
int motor_l_speed = -1;
int motor_r_speed = -1;

// bluetooth, pins 10, 11, 12, 13 (SPI) 14 = RST, 15 = RDY, south of Teensy mount
#include <SPI.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include <uart_over_ble.h>
#include <services.h>
#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
#define NUMBER_OF_PIPES 0
static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif
/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;
// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;
/* Temporary buffers for sending ACI commands */
static hal_aci_evt_t  aci_data;
//static hal_aci_data_t aci_cmd;

/* Timing change state variable */
static bool timing_change_done          = false;

/* Used to test the UART TX characteristic notification */
static uart_over_ble_t uart_over_ble;
static uint8_t uart_buffer[20];
static uint8_t uart_buffer_len = 0;
static uint8_t dummychar = 0;
bool stringComplete = false;  // whether the string is complete
uint8_t stringIndex = 0; //Initialize the index to store incoming chars

bool report_ready;
void report_(String msg) {  // switches between reporting over Serial to reporting over BLE
  Serial.print(msg);
}
void report_ble(String msg) {

}
void report_serial(String msg) {

}
void report(String msg) {
  report_serial(msg);
}

/* Initialize the radio_ack. This is the ack received for every transmitted packet. */
//static bool radio_ack_pending = false;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line) {
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

void uart_over_ble_init(void) {
    uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len) {
  bool status = false;
  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1)) {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status) {
      aci_state.data_credit_available--;
    }
  }
  return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length) {
  bool status = false;
  aci_ll_conn_params_t *conn_params;
  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) ) {
    Serial.println(*byte, HEX);
    switch(*byte) {
      /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
        Parameters:
        None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;

      /*
      Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte+1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                                conn_params->max_conn_interval,
                                conn_params->slave_latency,
                                conn_params->timeout_mult);
        status = true;
        break;

      /*
      Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;

      /*
      Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }
  return status;
}

void aci_loop() {
  static bool setup_required = false;
  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data)) {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode) {
      /** As soon as you reset the nRF8001 you will get an ACI Device Started Event */
      case ACI_EVT_DEVICE_STARTED: {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
            lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
            Serial.println(F("Advertising started"));
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe Number: "));
        Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          Serial.print(F(" Data(Hex) : "));
          for(int i=0; i<aci_evt->len - 2; i++)
          {
            char temp = (char)aci_evt->params.data_received.rx_data.aci_data[i];
            Serial.print(temp);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            if (temp == 's') {
              motor_l.drive(255,1000);
              motor_r.drive(255,1000);
            } else if (temp == 't') {
              motor_l.brake();
              motor_r.brake();
            }
          }

          uart_buffer_len = aci_evt->len - 2;
          Serial.println(F(""));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
          {
            /*Do this to test the loopback otherwise comment it out
            */
            /*
            if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
            {
              Serial.println(F("UART loopback failed"));
            }
            else
            {
              Serial.println(F("UART loopback OK"));
            }
            */
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
            uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;

    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}

// 2x ultrasonics, pins 1, 2, 16, 17 (5V tolerant), right of Teensy mount
#include <Ultrasonic.h>
Ultrasonic ultrasonic_1(1, 2); // (Trig PIN,Echo PIN), 5V tolerant pins
Ultrasonic ultrasonic_2(16, 17); // (Trig PIN,Echo PIN), 5V tolerant pins

// 10-DOF IMU, pins 19, 18 (I2C), right of Teensy mount
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
/* new sensor event */
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_event_t gyro_event;
sensors_vec_t   orientation;

void displaySensorDetails(void)
{
  sensor_t sensor;

  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  bmp.getSensor(&sensor);
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

// control button, status LED, pins 0, 20
#define CONTROL_BUTTON 0
#define STATUS_LED 20

String readString;
bool responsive = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("starting serial");

  /* Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001 */
  if (NULL != services_pipe_type_mapping) {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  } else {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin = 10; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin = 15; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin = 11;
  aci_state.aci_pins.miso_pin = 12;
  aci_state.aci_pins.sck_pin = 13;
  aci_state.aci_pins.reset_pin = 14; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin = UNUSED;
  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number = 1;
  aci_state.aci_pins.spi_clock_divider = SPI_CLOCK_DIV8;// SPI_CLOCK_DIV8 = 2MHz SPI speed, SPI_CLOCK_DIV16 = 1MHz SPI speed

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);

  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Found 10-DOF sensor");
}

void loop() {
  aci_loop();

  while (Serial.available()) {
    char c = Serial.read(); // gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2); //slow looping to allow buffer to fill with next character
  }
  if (readString.length() > 0) {
    Serial.println(readString);
  }

  readString = readString.trim();

  if (readString.length() <= 20 and readString.length() > 0) {
    uart_buffer_len = stringIndex + 1;
    for (int i = 0; i < readString.length(); ++i) {
      uart_buffer[i] = readString[i];
    }
    uart_buffer_len = readString.length();
    if (!lib_aci_send_data(
      PIPE_UART_OVER_BTLE_UART_TX_TX,
      uart_buffer,
      uart_buffer_len)) {
      Serial.println(F("Serial input dropped"));
    }
    // clear the uart_buffer:
    for (int i = 0; i < readString.length(); ++i) {
      uart_buffer[i] = ' ';
    }
  }

  if (readString == "s") {
    motor_l.drive(255,1000);
    motor_r.drive(255,1000);
  }
  else if (readString == "t") {
    motor_l.brake();
    motor_r.brake();
  }
  readString = "";

  // 10-DOF
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&accel_event);
  /*
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(accel_event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel_event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel_event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  */

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&mag_event);
  /*
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(mag_event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mag_event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mag_event.magnetic.z); Serial.print("  ");Serial.println("uT");
  */

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    // Serial.print(F("Heading: "));
    // Serial.print(orientation.heading);
    // Serial.print(F("; "));
  }

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&gyro_event);
  /*
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(gyro_event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print("  ");Serial.println("rad/s ");
  */

  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    // Serial.print(F("Roll: "));
    // Serial.print(orientation.roll);
    // Serial.print(F("; "));
    // Serial.print(F("Pitch: "));
    // Serial.print(orientation.pitch);
    // Serial.print(F("; "));
  }

  /* Use the new fusionGetOrientation function to merge accel/mag data */
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    // Serial.print(F("Orientation: "));
    // Serial.print(orientation.roll);
    // Serial.print(F(" "));
    // Serial.print(orientation.pitch);
    // Serial.print(F(" "));
    // Serial.print(orientation.heading);
    // Serial.println(F(""));
  }

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  // bmp.getEvent(&bmp_event);
  // if (bmp_event.pressure)
  // {
  //   /* Display atmospheric pressure in hPa */
  //   Serial.print(F("PRESS "));
  //   Serial.print(bmp_event.pressure);
  //   Serial.print(F(" hPa, "));
  //   /* Display ambient temperature in C */
  //   float temperature;
  //   bmp.getTemperature(&temperature);
  //   Serial.print(temperature);
  //   Serial.print(F(" C, "));
  //   /* Then convert the atmospheric pressure, SLP and temp to altitude    */
  //   /* Update this next line with the current SLP for better results      */
  //   float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  //   Serial.print(bmp.pressureToAltitude(seaLevelPressure,
  //                                       bmp_event.pressure,
  //                                       temperature));
  //   Serial.println(F(" m"));
  // }
  // Serial.println(F(""));
  //
  // // ultrasonics
  // Serial.print(ultrasonic_1.Ranging(CM)); // CM or INC
  // Serial.print(" cm, " );
  // Serial.print(ultrasonic_1.Timing());
  // Serial.println(" ms" ); // milliseconds

  delay(1000);
}
