// User interaction options

#define HEADLESS false // In headless mode, nothing is written to Serial0 (USB serial)
#define WAIT false // In wait is enabled, the system will halt until Serial0 connects.
uint16_t updateFrequency = 1000; // Dynamic polling rate
unsigned long counter = 0; // Message Counter

#define VBATPIN A7 // pin used to measure battery voltage via a 1/2 divider.

// Serial port pinmux for GPS comms, allows for multiple hardware-accelerated serial ports via SERCOM
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3
#define GPSECHO false // echo the GPS data to the Serial console?

// RFM69HCW Radio config
#define RF69_FREQ     915.0                       // radio frequency (MUST MATCH HARDWARE!)
#define RFM69_CS      6                           // chip select on pin D6
#define RFM69_INT     11                          // data ready interrupt on pin D11
#define RFM69_RST     5                           // radio soft-reset on pin D24

// SD card configuration
#define SD_CS 4                                   // chip select on pin D4

// Built in Arduino libraries
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "wiring_private.h" // access to private pinPeripheral() function

// External libraries (all available via library manager as of 10/18)
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <RH_RF69.h>

#include "radioEncode.h" // custom radio packet structure

// Radio object
RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

// Sensor objects
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

// define the GPS's hardware serial port
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
// Interrupt handler for Serial2
void SERCOM1_Handler() { Serial2.IrqHandler(); }
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial); // Init the GPS handler object with the assigned serial port

// Data logging SD card configuration
Sd2Card card;
SdVolume volume;
SdFile root;
bool card_available = false;

// Give the barometic sensor object the appropriate SLP for altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// This timer is used in the loop() method to service the sensor checking
unsigned long timer = millis();

// set onDemand to true when the user requests an update, and returned to false after update has been serviced
bool onDemand = false; 
