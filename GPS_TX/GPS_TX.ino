/*
 * Sketch for transmitting GPS data to RF.
 * Adafruit M0
 */

#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF69.h>
#include "wiring_private.h" // pinPeripheral() function

#include "config.h"
#include "radioEncode.h"

// Radio 
RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

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

uint32_t timer = millis();

/*
 * SETUP method for Arduino
 */
void setup() {
  // Set IO pin modes
  pinMode(LED_BUILTIN, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  // Assign pins to SERCOM functionality
  pinPeripheral(PIN_SERIAL2_TX, PIO_SERCOM);
  pinPeripheral(PIN_SERIAL2_RX, PIO_SERCOM);

  if (!HEADLESS) {
    if (WAIT) while (!Serial);  // hold the system hostage until it's time to start
    Serial.begin(115200); // talk to the host at a brisk rate, so we have time to write without dropping chars
    Serial.println("Let's Start!");
  }
  rfInitialize();
  gpsInitialize();
}

/*
 * Program Loop
 */
void loop() {
  
  // pull a chars one at a time from the GPS
  char c = GPS.read();
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse; wait for another message
  }
  // if millis() or timer wraps around, we'll reset it
  if (timer > millis()) timer = millis();

  // wait around for the update time
  if (millis() - timer > UPDATE_FREQ * 1000) {

    struct statusStruct radioPacket;
    struct statusStruct* packetPtr = &radioPacket;
    Adafruit_GPS* gpsPtr = &GPS;
    buildPacket(gpsPtr, packetPtr);
    
    // Send data to RF
    rf69.send((uint8_t*)packetPtr, sizeof(radioPacket));
    rf69.waitPacketSent();

    // blink LED to show activity
    blink(LED_BUILTIN, 1, 50);

    if(!HEADLESS){
      Serial.print("Packet {"); 
      for (int i = 0; i < sizeof(radioPacket); i++) {
        Serial.print(((uint8_t *)packetPtr)[i]); Serial.print(' ');
      }
      Serial.println("}"); 
    }
    
    outputToSerial();
    
    timer = millis(); // reset the timer
  }
}

// Output to serial if HEADLESS is false
void outputToSerial(){
  if (!HEADLESS) {
      Serial.print("Time: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      
      Serial.print(GPS.year, DEC); Serial.print("/");
      Serial.print(GPS.month, DEC); Serial.print('/');
      Serial.println(GPS.day, DEC);
      
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.print((int)GPS.fixquality);
      Serial.print(", Satellites: "); Serial.println((int)GPS.satellites);
      if (GPS.fix) {
        //Serial.println(GPS.latitude_fixed);
        //Serial.println(GPS.longitude_fixed);
        // Pull out the whole and decimal components of the lat and long for display
        int latWhole = GPS.latitude_fixed / 10000000;
        int latDec = GPS.latitude_fixed % 10000000;
        int lonWhole = GPS.longitude_fixed / 10000000;
        int lonDec = GPS.longitude_fixed % 10000000;
        
        Serial.print("Location: ");
        if(GPS.lat == 'S') Serial.print("-");
        Serial.print(latWhole); Serial.print("."); Serial.print(latDec);
        Serial.print(", ");
        if(GPS.lon == 'W') Serial.print("-");
        Serial.print(lonWhole); Serial.print("."); Serial.println(lonDec);
        
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        
      }
      Serial.println("-");
    }
}

// toggle the given pin by some count # of times for a duration of 'wait'
void blink(int pin, int count, int wait) {
  bool state = false;
  for (int i = 0; i < count * 2; i++) {
    digitalWrite(pin, state?HIGH:LOW);
    state = !state;
    delay(wait);
  }
}

// GPS Initialize Method
void gpsInitialize(){
  if (!HEADLESS) Serial.print("Waiting for GPS serial... ");
  while (!Serial2);  // hold the system until GPS serial port is ready
  if (!HEADLESS) Serial.println("It's up!");
  GPS.begin(9600);  // 9600 NMEA is the default baud rate for Adafruit MTK GPS
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status too
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000); // Wait a full second (!) for the GPS to be ready for us
  GPSSerial.println(PMTK_Q_RELEASE);
}

// RF69 Initialize Method
void rfInitialize(){
  if (!HEADLESS) Serial.print("Resetting Radio... ");
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    if (!HEADLESS) Serial.println("RFM69 radio init failed!");
    while (1); // halt
  }
  
  if (!HEADLESS) Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module) and no encryption.
  if (!rf69.setFrequency(RF69_FREQ)) {
    if (!HEADLESS) Serial.println("setFrequency failed");
    while (1); // halt
  }

  rf69.setTxPower(20, true);  // set broadcast power (valid range 14-20), 2nd arg always true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  if (!HEADLESS) { Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz"); }
}

// Set up the SD card and log file
bool sdInitialize(size_t CSpin) {
  if (!card.init()) {
    if (!HEADLESS) Serial.println("SD card initialization failed. Logging Disabled.");
  } else {
    if (!HEADLESS) Serial.println("SD Card detected and writable. Logging Enabled.");
    card_available = true;
  }
}
