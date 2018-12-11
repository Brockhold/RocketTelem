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
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void usageMessage() {
  Serial.print(" | To set a polling rate enter a value between "); Serial.print(MINPERIOD);
  Serial.println("ms and 10000ms into the Serial input.");
  Serial.println(" |     Entering '0' or <emptystring> returns the current status and enters on-demand mode.");
  Serial.println(" |     Sending 's 0' and 's 1' enables or disables sd logging respectively.");
  Serial.println(" |     Sending 's 2' creates a new log file. Sending 's 9' erases the current log.");
}

// RF Initialize Method
void rfInitialize(){
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("[!] RFM69 radio init failed");
    while (1);
  }
  Serial.println("[!] RFM69 radio init OK!");
  
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

  Serial.print("[!] RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

bool btInit() {
  ble.begin(VERBOSE_MODE);
  ble.factoryReset();
  ble.echo(false);
  ble.verbose(false);
  
  ble.setMode(BLUEFRUIT_MODE_DATA);
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

    //count of commands received
    ++terminalCounter;
    Serial.print("[!] Terminal command received. Current Count: ");
    Serial.println(terminalCounter);

    if(Serial.peek() == 's'){
      polling s;
      char sdCard = Serial.read(); //remove the s
      uint8_t sdCommand = Serial.parseInt(); //parse the number
      Serial.print("Sending SD Card update: "); Serial.print(sdCard); Serial.print(" "); Serial.println(sdCommand);
      s.sdCommand = sdCommand;
      s.message_id = counter;
      s.sdCard = sdCard;
      
      ++counter;
      rf69.send((uint8_t *)&s, sizeof(s));
      rf69.waitPacketSent();
      
      clearBuffer();
      usageMessage();
      return;
    }
    
    polling p;
    p.polling_rate = Serial.parseInt();
    p.message_id = counter;
    
    // clears out buffer for next read
    clearBuffer();

    // check validity of polling rate
    if ((p.polling_rate == 0) || (p.polling_rate >= MINPERIOD)) {
      // Send polling rate to TX
      ++counter;
      rf69.send((uint8_t *)&p, sizeof(p));
      rf69.waitPacketSent();
      
      // tell the user about the new polling rate, and optionally the usage
      if(p.polling_rate == 0){
        Serial.println("[!] Requested On-Demand update(s)");
        usageMessage();
      } else {
        Serial.print("[!] Polling rate has been updated to: ");
        Serial.print(p.polling_rate); Serial.println("ms");
      }
    } else {
      Serial.print(p.polling_rate); Serial.println("ms is too fast, please enter another value.");
      delay(2000);
    }
  }
}

void readBTInput() {
  //send 0 or <emptystring> for on-demand mode. press enter to receive data.
  if(ble.available()){

    //count of commands received
    ++terminalCounter;
    ble.print("[!] BT command received. Current Count: ");
    ble.println(terminalCounter);

    if(ble.peek() == 's'){
      polling s;
      char sdCard = ble.read(); //remove the s
      uint8_t sdCommand = ble.parseInt(); //parse the number
      ble.print("Sending SD Card update: "); ble.print(sdCard); ble.print(" "); ble.println(sdCommand);
      s.sdCommand = sdCommand;
      s.message_id = counter;
      s.sdCard = sdCard;
      
      ++counter;
      rf69.send((uint8_t *)&s, sizeof(s));
      rf69.waitPacketSent();
      
      clearBuffer();
      usageMessage();
      return;
    }
    
    polling p;
    p.polling_rate = ble.parseInt();
    p.message_id = counter;
    
    // clears out buffer for next read
    clearBuffer();

    // check validity of polling rate
    if ((p.polling_rate == 0) || (p.polling_rate >= MINPERIOD)) {
      // Send polling rate to TX
      ++counter;
      rf69.send((uint8_t *)&p, sizeof(p));
      rf69.waitPacketSent();
      
      // tell the user about the new polling rate, and optionally the usage
      if(p.polling_rate == 0){
        ble.println("[!] Requested On-Demand update(s)");
        usageMessage();
      } else {
        ble.print("[!] Polling rate has been updated to: ");
        ble.print(p.polling_rate); ble.println("ms");
      }
    } else {
      ble.print(p.polling_rate); ble.println("ms is too fast, please enter another value.");
      delay(2000);
    }
  }
}

/**
 * Sends received packet data to Serial
 */
void displayPacketData(statusStruct &packet){
  Serial.println("<packet>");
  Serial.println("  Metadata:");

  #if DEBUG
    Serial.print("    Raw = {"); 
    for (int i = 0; i < sizeof(packet); i++) {
      Serial.print(' '); Serial.print(((uint8_t *)&packet)[i]); 
    }
    Serial.println("}");
  #endif

  Serial.print("    Message ID: "); Serial.println(packet.message_id);
  Serial.print("    Current Polling Rate: ");
  if(packet.polling_rate < 2){
   Serial.println("On-Demand");
  }else{
   Serial.print(packet.polling_rate); Serial.println("ms");
  }
  Serial.print("    Logging Status: "); Serial.println(packet.sdStatus ? "Enabled" : "Disabled");
  
  Serial.print("  Date & Time:    "); 
  // Year/Month/Day
  Serial.print(packet.year); Serial.print("/");
  Serial.print(packet.month); Serial.print("/");
  Serial.print(packet.day);
  Serial.print("    ");
  // Hour:Minute:Second
  Serial.print(packet.hour); Serial.print(":");
  Serial.print(packet.minute); Serial.print(":");
  Serial.println(packet.seconds);

  Serial.println("  Transmitter Status:");
  
  // Transmitter battery level
  Serial.print("    Battery voltage: "); Serial.print((float)packet.batt_level / 1024); Serial.println("v");
  Serial.print("    Temperature (C): "); Serial.println(packet.temperature);

  Serial.println("  Physical Status:");
  
  // GPS reception status
  Serial.print("    GPS fix: "); Serial.print(packet.fix?"Yes":"No");
  Serial.print(", Quality: "); Serial.print(packet.fixquality);
  Serial.print(", Satellites: "); Serial.println(packet.satellites);

    // If there is a GPS fix, print the reported Lat & Ln
    if (packet.fix) {
      Serial.print("    Location: ");
      if ((char) packet.lat == 'S') Serial.print("-");
      Serial.print(packet.latitude_fixed/10000000); Serial.print("."); Serial.print(packet.latitude_fixed % 10000000);
      Serial.print(", ");
      if ((char) packet.lon == 'W') Serial.print("-");
      Serial.print(packet.longitude_fixed/10000000); Serial.print("."); Serial.println(packet.longitude_fixed % 10000000);
    }
    // Barometric altimeter status
    Serial.print("    Altitude: "); Serial.println(packet.bar_alt);
    // Accelerometer status
    Serial.print("    Accelerometer {"); 
    Serial.print(" Pitch: "); Serial.print((float)(packet.pitch) / 700); 
    Serial.print(" Roll: "); Serial.print((float)(packet.roll) / 300);
    Serial.print(" Heading: "); Serial.print((float)(packet.heading) / 180);
    Serial.println(" }");
    //Serial.print("Compass Heading: "); Serial.println(packet.mag_heading);
    
    Serial.println("</packet>"); Serial.println();
 }

void btPrint(statusStruct &packet) {
  ble.print("Packet ID: "); ble.print(packet.message_id);
  ble.print(" rate: ");
  if(packet.polling_rate < 2){
   ble.println("On-Demand");
  }else{
   ble.print(packet.polling_rate); ble.println("ms");
  }
  ble.print("Log: "); ble.println(packet.sdStatus ? "Yes" : "No");
  
  // Year/Month/Day
  ble.print(packet.year); ble.print("/");
  ble.print(packet.month); ble.print("/");
  ble.print(packet.day);
  ble.print("    ");
  // Hour:Minute:Second
  ble.print(packet.hour); ble.print(":");
  ble.print(packet.minute); ble.print(":");
  ble.println(packet.seconds);

  // Transmitter battery level
  ble.print((float)packet.batt_level / 1024); ble.println("v");
  ble.print(packet.temperature); ble.println("c"); 

  // GPS reception status
  ble.print("Fix: "); ble.print(packet.fix?"Yes":"No");
  ble.print(", Q: "); ble.print(packet.fixquality);
  ble.print(", Sats: "); ble.println(packet.satellites);

    // If there is a GPS fix, print the reported Lat & Ln
    if (packet.fix) {
      ble.print("Loc: ");
      if ((char) packet.lat == 'S') ble.print("-");
      ble.print(packet.latitude_fixed/10000000); ble.print("."); ble.print(packet.latitude_fixed % 10000000);
      ble.print(", ");
      if ((char) packet.lon == 'W') ble.print("-");
      ble.print(packet.longitude_fixed/10000000); ble.print("."); ble.println(packet.longitude_fixed % 10000000);
    }
    // Barometric altimeter status
    ble.print("Alt: "); Serial.println(packet.bar_alt);
    // Accelerometer status
    ble.print("Accel{"); 
    ble.print(" P: "); ble.print((float)(packet.pitch) / 700); 
    ble.print(" R: "); ble.print((float)(packet.roll) / 300);
    ble.print(" H: "); ble.print((float)(packet.heading) / 180);
    ble.println(" }\n");
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

  rfInitialize();
  btInit();
  usageMessage();
}

/*
 * Program Loop
 */

void loop() {
  
  readInput(); // check for new polling rate directive from user
  //readBTInput();
  struct statusStruct radioPacket;
  uint8_t len = sizeof(radioPacket);
  if (rf69.available() && rf69.recv((uint8_t *)&radioPacket, &len)) {
    if (!len) return;

    // blink LED to show activity
    blink(LED_BUILTIN, 1, 50);

    Serial.print("Received ["); 
    Serial.print(len); 
    Serial.print("] @RSSI ");
    Serial.println(rf69.lastRssi(), DEC);

    displayPacketData(radioPacket);
    btPrint(radioPacket);
  }
}

void clearBuffer(){
  while(Serial.available() > 0){
      Serial.read();
    }
}
