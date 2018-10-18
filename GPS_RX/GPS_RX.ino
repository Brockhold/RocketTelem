/*
 * Sketch for receiving GPS data from RF.
 * Adafruit M0
 */

#include <SPI.h>
#include <RH_RF69.h>

#include "config.h"
#include "radioDecode.h"


RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

/*
 * SETUP
 */
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);   
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  rfInitialize();
}

/*
 * Program Loop
 */
void loop() {
  
  struct statusStruct radioPacket;
  struct statusStruct* packetPtr = &radioPacket;
  
  uint8_t len = sizeof(radioPacket);
  if (rf69.available() && rf69.recv((uint8_t *)packetPtr, &len)) {
    if (!len) return;

    // blink LED to show activity
    blink(LED_BUILTIN, 1, 50);

    Serial.print("Received ["); 
    Serial.print(len); 
    Serial.print("] @RSSI ");
    Serial.print(rf69.lastRssi(), DEC);

    Serial.print("{"); 
//    for (int i = 0; i < len; i++) {
//      Serial.print(' '); Serial.print(radioPacket[i]); 
//    }
    Serial.println("}");

    Serial.print("Time: "); // Hour:Minute:Second
    Serial.print(radioPacket.hour); Serial.print(":");
    Serial.print(radioPacket.minute); Serial.print(":");
    Serial.println(radioPacket.seconds);

    Serial.print("Date: "); // Year/Month/Day
    Serial.print(radioPacket.year); Serial.print("/");
    Serial.print(radioPacket.month); Serial.print("/");
    Serial.println(radioPacket.day);

    // GPS reception status
    Serial.print("Fix: "); Serial.print(radioPacket.fix?"Yes":"No");
    Serial.print(", Quality: "); Serial.print(radioPacket.fixquality);
    Serial.print(", Satellites: "); Serial.println(radioPacket.satellites);

    // If there is a GPS fix, print the reported Lat & Ln
    if (radioPacket.fix) {
      Serial.print("Location: ");
      if ((char) radioPacket.lat == 'S') Serial.print("-");
      Serial.print(radioPacket.latitude_fixed/10000000); Serial.print("."); Serial.print(radioPacket.latitude_fixed % 10000000);
      Serial.print(", ");
      if ((char) radioPacket.lon == 'W') Serial.print("-");
      Serial.print(radioPacket.longitude_fixed/10000000); Serial.print("."); Serial.println(radioPacket.longitude_fixed % 10000000);
      
      Serial.print("Speed (knots): "); Serial.print(radioPacket.speed);
      Serial.print(", Angle: "); Serial.print(radioPacket.angle);
      Serial.print(", Altitude: "); Serial.println(radioPacket.altitude);
    }
    Serial.println("-");
  }
}

// RF Initialize Method
void rfInitialize(){
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  Serial.println();
  Serial.println("Waiting for message...");
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
