#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "wiring_private.h" // pinPeripheral() function

#define HEADLESS false
#define WAIT false
#define UPDATE_FREQ 1

// Serial port pinmux for GPS comms
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3

Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
#define GPSSerial Serial2
// Connect to the GPS on the hardware serial port
Adafruit_GPS GPS(&GPSSerial);

// echo the GPS data to the Serial console?
#define GPSECHO false

// RFM69HCW Radio config
#define RF69_FREQ 915.0
#define RFM69_CS      6
#define RFM69_INT     11
#define RFM69_RST     5

#define PACKET_LEN    22

RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

int16_t packetnum = 0;  // packet counter, we increment per xmission
uint32_t timer = millis();

// Interrupt handler for Serial2
void SERCOM1_Handler() { Serial2.IrqHandler(); }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  
  digitalWrite(RFM69_RST, HIGH); // Perform a reset on the radio
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!HEADLESS & !rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1); // halt
  }
  if (!HEADLESS) Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    if (!HEADLESS) Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // set broadcast power (valid range 14-20), 2nd arg always true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  if (!HEADLESS) { Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz"); }
  
  // Assign pins to SERCOM functionality
  pinPeripheral(PIN_SERIAL2_TX, PIO_SERCOM);
  pinPeripheral(PIN_SERIAL2_RX, PIO_SERCOM);

  if (WAIT) {
  while (!Serial);  //wait until host Serial is ready
    Serial.begin(115200); // talk to the host at a brisk rate, so we have time to write without dropping chars
    Serial.println("Adafruit GPS library basic test!");
    Serial.println("Waiting for GPS serial...");
  }
  while (!Serial2);  // wait for GPS serial
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
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000); // Wait a second for the GPS to be ready for us
  GPSSerial.println(PMTK_Q_RELEASE);
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

void loop() {
  // pull a chars one at a time from the GPS
  char c = GPS.read();
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // wait around for the update time
  if (millis() - timer > UPDATE_FREQ * 1000) {

    /*
     * 1 byte to hold a boolean for the "fix" value -- TODO pack better?
     * 2 bytes for the uint8_t
     *    fixquality, satelites
     * 
     * 7 bytes of the uint8_t values: 
     *    Year, Month, Day, Hour, Minute, Second
     * 
     * 8 bytes of two int32_t values (1/10000 degree increments):
     *    latitude_fixed, longitude_fixed;
     * 
     * 2 Bytes of the chars:
     *    lat, lon
     * 
     */
    int pktpos = 0;
    uint8_t radiopacket[PACKET_LEN];

    radiopacket[pktpos++] = GPS.fix?1:0;
    radiopacket[pktpos++] = GPS.fixquality;
    radiopacket[pktpos++] = GPS.satellites;
    radiopacket[pktpos++] = GPS.year;
    radiopacket[pktpos++] = GPS.month;
    radiopacket[pktpos++] = GPS.day;
    radiopacket[pktpos++] = GPS.hour;
    radiopacket[pktpos++] = GPS.minute;
    radiopacket[pktpos++] = GPS.seconds;
    radiopacket[pktpos+=4] = GPS.latitude_fixed;
    radiopacket[pktpos+=4] = GPS.longitude_fixed;
    radiopacket[pktpos++] = GPS.lat;
    radiopacket[pktpos++] = GPS.lon;
    
    if (!HEADLESS) { 
      Serial.print("Packet "); 
      for (int i = 0; i < PACKET_LEN; i++) {
        Serial.print(radiopacket[i]); Serial.print(' ');
      }
      Serial.println(""); 
    }
    rf69.send(radiopacket, PACKET_LEN);
    rf69.waitPacketSent();
    blink(LED_BUILTIN, 1, 10);
    
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
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
      Serial.println("-");
    }
    
    timer = millis(); // reset the timer
  }
}
