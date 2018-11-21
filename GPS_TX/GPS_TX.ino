/*
 * Sketch for transmitting sensor data via radio.
 * Designed to run on an Adafruit M0.
 * See config.h for a list of pins connected to hardware and
 * a list of all necessary Arduino libraries.
 */

// Configuration files and helper methods
#include "config.h"

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
    Serial.print("Packet size: "); Serial.print(sizeof(statusStruct)); Serial.println(" bytes");
  }
  rfInitialize();
  gpsInitialize();
  initSensors();
}

/*
 * Program Loop
 */
void loop() {
  
  // pull a chars one at a time from the GPS
  GPS.read();
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse; wait for another message
  }
  
  // if millis() or timer wraps around, we'll reset it
  if (timer > millis()) timer = millis();

  // wait around for the sensor update time
  if (millis() - timer >= updateFrequency) {

    if(updateFrequency > 0){

      // Build the radio packet to be sent
      statusStruct radioPacket = getSensorData();
      
      // Send data to RF
      rf69.send((uint8_t*)&radioPacket, sizeof(radioPacket));
      rf69.waitPacketSent();
  
      // blink LED to show activity
      blink(LED_BUILTIN, 1, 50);
  
  #if !HEADLESS
  
        Serial.print("Packet {"); 
        for (int i = 0; i < sizeof(radioPacket); i++) {
          Serial.print(((uint8_t *)&radioPacket)[i]); Serial.print(' ');
        }
        Serial.println("}"); 
        
  #endif
      
      outputToSerial(&radioPacket);
      
      timer = millis(); // reset the timer
    
      if(updateFrequency == 1) updateFrequency = 0; // reset on-demand
    }
    
  }

  checkPollingUpdate();
  
}

// Gets all the sensor data and returns the struct
// If any even continues to fail the whole program fails
statusStruct getSensorData(){
  // Event structures for each sensor
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t bmp_event;
    sensors_vec_t   orientation; // vector of floats for pitch/roll/heading

    uint8_t retry = 3;
    // these may fail...if either of them do retry
    // get sensor events
    getSensorEvents(&accel_event, &mag_event, &bmp_event);

    // fill out the orientation vector struct (floats) with accel data
    dof.accelGetOrientation(&accel_event, &orientation);

    // add magnetic heading to orientation struct
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
    
    // get temperature as a float and then force into integer representaton
    float temp_float;
    bmp.getTemperature(&temp_float);
    int16_t temp_int = temp_float;

    // get barometric pressure and convert to altitude
    int16_t altitude_int = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, bmp_event.pressure, temp_float);

    ++counter; // Increase message counter for tracking packets

    // take all those structures and copy their values into the radio packet struct
    return buildPacket(orientation, altitude_int, temp_int, &GPS, updateFrequency);
}

// Checks to see if a polling packet was received.
// updates the polling frequency based on the received
// value. Values is in milliseconds and contrained 
// between 100 and 10000.
void checkPollingUpdate(){
  
  polling p;
  uint8_t len = sizeof(p);

  // check to see if the receiver sent a new poll rate
  if(rf69.available() && rf69.recv((uint8_t *)&p, &len)){
    
#if !HEADLESS
    Serial.print("Received [");
    Serial.print(len);
    Serial.print("] Message ID: ");
    Serial.print(p.message_id);
    Serial.print("Value: ");
    Serial.println(p.polling_rate);
#endif

    if(p.polling_rate == 0 || p.polling_rate == 1){
      updateFrequency = p.polling_rate;
      return;
    }
    // Constrain the value and update the frequency
    updateFrequency = constrain(p.polling_rate, 100, 10000);
  }
}

// Output to serial if HEADLESS is false
void outputToSerial(StatusStruct* message){
  if (!HEADLESS) {
      Serial.print("Message ID: ");
      Serial.print(message->message_id);
      Serial.print(" Current Polling Rate: ");
      Serial.print(message->polling_rate); Serial.println("ms");
      Serial.print("Time: ");
      Serial.print(message->hour, DEC); Serial.print(':');
      Serial.print(message->minute, DEC); Serial.print(':');
      Serial.println(message->seconds, DEC);
      
      Serial.print("Date: ");
      Serial.print(message->year, DEC); Serial.print("/");
      Serial.print(message->month, DEC); Serial.print('/');
      Serial.println(message->day, DEC);
      
      Serial.print("Fix: "); Serial.print(message->fix);
      Serial.print(" quality: "); Serial.print(message->fixquality);
      Serial.print(", Satellites: "); Serial.println(message->satellites);
      if (message->fix > 0) {
        // Pull out the whole and decimal components of the lat and long for display
        int latWhole = message->latitude_fixed / 10000000;
        int latDec = message->latitude_fixed % 10000000;
        int lonWhole = message->longitude_fixed / 10000000;
        int lonDec = message->longitude_fixed % 10000000;
        
        Serial.print("Location: ");
        if(message->lat == 'S') Serial.print("-");
        Serial.print(latWhole); Serial.print("."); Serial.print(latDec);
        Serial.print(", ");
        if(message->lon == 'W') Serial.print("-");
        Serial.print(lonWhole); Serial.print("."); Serial.println(lonDec);
      }
      
      // printouts for temperature & altitude
      Serial.print("Temperature: "); Serial.print(message->temperature); Serial.print("ºC\t");
      Serial.print("Baro Alt: "); Serial.print(message->bar_alt); Serial.println("m");
      // printouts for pitch/roll/heading
      Serial.print("Pitch/Roll/Heading: "); 
      Serial.print(message->pitch); Serial.print("/");
      Serial.print(message->roll); Serial.print("/");
      Serial.println(message->heading);
      
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

// start up the 10-dof sensors 
void initSensors() {
  if (!accel.begin() || !mag.begin() || !bmp.begin()) {
    if (!HEADLESS) Serial.println("unable to init sensors! Halted");
    while(true); // halt
  }
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

// This gathers the sensor data. If any of the sensors fail to get an event a few times the system fails
void getSensorEvents(sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_event_t *bmp_event){
  uint8_t retry = 3;
  while(!accel.getEvent(accel_event)){
    if(retry < 1) exit(1);
    --retry;
  }
  retry = 3;
  while(!mag.getEvent(mag_event)){
    if(retry < 1) exit(1);
    --retry;
  }
  retry = 3;
  while(!bmp.getEvent(bmp_event)){
    if(retry < 1) exit(1);
    --retry;
  }
}
