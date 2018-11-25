/*
 * Sketch for receiving sensor data from remote device..
 * 
 * Hardware: 
 *  Adafruit M0 (currently using BTLE Bluefruit, but not using the BT module)
 *  RF69HCW radio tranceiver connected to SPI (see config.h for pin details)
 */

#include "config.h"
#include "radioDecode.h"

RH_RF69 rf69(RFM69_CS, RFM69_INT); // radio driver instance

void usageMessage() {
  Serial.println("To change polling rate enter a value between 100 and 10000 into the Serial input.");
  Serial.println(" Entering '0' or <emptystring>  places the transmitter into on-demand mode and returns the current status.");
  Serial.println("Sending 'sd 0' and 'sd 1' enables and disables sd logging respectively");
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

/**
 * Reads integer from Serial to set polling rate on TX
 */
void readInput(){
  //send 0 or <emptystring> for on-demand mode. press enter to receive data.
  if(Serial.available() > 0){
    polling p;
    p.polling_rate = Serial.parseInt();
    p.message_id = counter;
    
    // clears out buffer for next read
    while(Serial.available() > 0){
      Serial.read();
    }

    // print details to screen
    Serial.println();
    Serial.print("Polling rate has been updated to: ");
    
    if (!(p.polling_rate < MINPERIOD)) {
      if(p.polling_rate == 0){
        Serial.println("On-Demand");
        usageMessage();
      } else {
        Serial.print(p.polling_rate); Serial.println("ms");
      }
      // Send data to RF
      ++counter;
      rf69.send((uint8_t *)&p, sizeof(p));
      rf69.waitPacketSent();
    } else {
      Serial.print(p.polling_rate); Serial.println("ms is too fast, please enter another value.");
    }
  }
}

/**
 * Sends received packet data to Serial
 */
void displayPacketData(statusStruct &packet){

#if DEBUG

      Serial.print("Raw Packet Data = {"); 
      for (int i = 0; i < sizeof(packet); i++) {
        Serial.print(' '); Serial.print(((uint8_t *)&packet)[i]); 
      }
      Serial.println("}");

#endif

   Serial.print("Message ID: ");
   Serial.print(packet.message_id);
   Serial.print(" Current Polling Rate: ");
   if(packet.polling_rate < 2){
    Serial.println("On-Demand");
   }else{
    Serial.print(packet.polling_rate); Serial.println("ms");
   }
   Serial.print("Time: "); // Hour:Minute:Second
    Serial.print(packet.hour); Serial.print(":");
    Serial.print(packet.minute); Serial.print(":");
    Serial.println(packet.seconds);

    Serial.print("Date: "); // Year/Month/Day
    Serial.print(packet.year); Serial.print("/");
    Serial.print(packet.month); Serial.print("/");
    Serial.println(packet.day);

    // GPS reception status
    Serial.print("Fix: "); Serial.print(packet.fix?"Yes":"No");
    Serial.print(", Quality: "); Serial.print(packet.fixquality);
    Serial.print(", Satellites: "); Serial.println(packet.satellites);

    // If there is a GPS fix, print the reported Lat & Ln
    if (packet.fix) {
      Serial.print("Location: ");
      if ((char) packet.lat == 'S') Serial.print("-");
      Serial.print(packet.latitude_fixed/10000000); Serial.print("."); Serial.print(packet.latitude_fixed % 10000000);
      Serial.print(", ");
      if ((char) packet.lon == 'W') Serial.print("-");
      Serial.print(packet.longitude_fixed/10000000); Serial.print("."); Serial.println(packet.longitude_fixed % 10000000);
    }
    
    Serial.print("Temperature (C): "); Serial.println(packet.temperature);
    Serial.print("Accelerometer {"); 
      Serial.print(" Pitch: "); Serial.print(packet.pitch); 
      Serial.print(" Roll: "); Serial.print(packet.roll);
      Serial.print(" Heading: "); Serial.print(packet.heading);
      Serial.println(" }");
    //Serial.print("Compass Heading: "); Serial.println(packet.mag_heading);
    Serial.print("Altitude: "); Serial.println(packet.bar_alt);

    Serial.println("-");
 }

 /*
 * SETUP
 */
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  usageMessage();
  delay(5000); // wait five seconds before continuing

  rfInitialize();
}

/*
 * Program Loop
 */
void loop() {
  
  readInput(); // check for new polling rate directive from user


  if (rf69.available()) {
    struct statusStruct radioPacket;
    uint8_t len = 0;
     if (rf69.recv((uint8_t *)&radioPacket, &len)) {
      // first validate the length of the packet is equal to the storage size
      if (len == sizeof(radioPacket)) {
        // blink LED to show activity
        blink(LED_BUILTIN, 1, 50);
    
        Serial.print("Received ["); 
        Serial.print(len); 
        Serial.print("] @ RSSI ");
        Serial.println(rf69.lastRssi(), DEC);
        
        displayPacketData(radioPacket);
      } else {
        // case in which length was too small after valid receive
      }
    } else {
      // case in which a packet was available but the recv failed
    }
  } else {
    // no new packet was available
  }
}
