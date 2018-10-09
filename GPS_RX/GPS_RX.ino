/*
 * Sketch for receiving GPS data from RF.
 * Adafruit M0
 */

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/
#define RF69_FREQ 915.0
#define RFM69_CS      6
#define RFM69_INT     11
#define RFM69_RST     12
#define PACKET_LEN    24    // Expected packet size

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
  // Should be a message for us now   
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf69.available() && rf69.recv(buf, &len)) {
    if (!len) return;

    // blink LED to show activity
    blink(LED_BUILTIN, 1, 50);

    Serial.print("Received ["); 
    Serial.print(len); 
    Serial.print("] @RSSI ");
    Serial.print(rf69.lastRssi(), DEC);

    Serial.print("{"); 
    for (int i = 0; i < PACKET_LEN; i++) {
      Serial.print(' '); Serial.print(buf[i]); 
    }
    Serial.println("}");

    Serial.print("Time: "); // Hour:Minute:Second
    Serial.print(buf[6]); Serial.print(":");
    Serial.print(buf[7]); Serial.print(":");
    Serial.println(buf[8]);

    Serial.print("Date: "); // Year/Month/Day
    Serial.print(buf[3]); Serial.print("/");
    Serial.print(buf[4]); Serial.print("/");
    Serial.println(buf[5]);

    // GPS reception status
    Serial.print("Fix: "); Serial.print((buf[0] == 1)?"Yes":"No");
    Serial.print(", Quality: "); Serial.print(buf[1]);
    Serial.print(", Satellites: "); Serial.println(buf[2]);

    // If there is a GPS fix, print the reported Lat & Ln
    if (buf[0] == 1) {
      // Fixed decimal representation of the lat and lon, in 1/100000 degrees
      uint32_t lat_fixed = buf[9] *10000000 + buf[10] * 100000 + buf[11] * 1000 + buf[12] * 10 + buf[13];
      uint32_t lon_fixed = buf[14]*10000000 + buf[15] * 100000 + buf[16] * 1000 + buf[17] * 10 + buf[18];
      
      Serial.print("Location: ");
      if ((char) buf[19] == 'S') Serial.print("-");
      Serial.print(lat_fixed/10000000); Serial.print("."); Serial.print(lat_fixed % 10000000);
      Serial.print(", ");
      if ((char) buf[20] == 'W') Serial.print("-");
      Serial.print(lon_fixed/10000000); Serial.print("."); Serial.println(lon_fixed % 10000000);
      
      Serial.print("Speed (knots): "); Serial.print(buf[21]);
      Serial.print(", Angle: "); Serial.print(buf[22]);
      Serial.print(", Altitude: "); Serial.println(buf[23]);
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
