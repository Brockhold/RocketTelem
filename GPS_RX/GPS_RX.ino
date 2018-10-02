#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/
#define RF69_FREQ 915.0

#define RFM69_CS      6
#define RFM69_INT     11
#define RFM69_RST     12

RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED_BUILTIN, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

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
  Serial.println("Waiting for messages...");
}


void loop() {
 if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
  
      Serial.print("Fix: "); Serial.println((buf[0] == 1)?"Yes":"No");
      Serial.print("Quality:"); Serial.println(buf[1]);
      Serial.print("Satellites:"); Serial.println(buf[2]);

      Serial.print("Year:"); Serial.println(buf[3]);
      Serial.print("Month:"); Serial.println(buf[4]);
      Serial.print("Day:"); Serial.println(buf[5]);
      Serial.print("Hour:"); Serial.println(buf[6]);
      Serial.print("Minute:"); Serial.println(buf[7]);
      Serial.print("Second:"); Serial.println(buf[8]);
      
      //Serial.print("Lat:"); Serial.println(buf[9]);
      //Serial.print("Lon:"); Serial.println(buf[]);
      //Serial.print("Satellites:"); Serial.println(buf[2]);
      //Serial.print("Satellites:"); Serial.println(buf[2]);
      
    } else {
      Serial.println("Receive failed");
    }
  }
}
