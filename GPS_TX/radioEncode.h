/*
 * Defined struct for holding all sensor data that the radio will transfer
 * the maximum size of a single radio packet is 64 bytes, so keep track of usage
 */
struct statusStruct {
  boolean fix;
  // 1 byte (?)
  uint8_t lat, lon, mag, fixquality, satellites, hour, minute, seconds, year, month, day; 
  // 12 bytes
  uint16_t milliseconds;
  // 14 bytes
  int32_t latitude_fixed, longitude_fixed; // 8 bytes
  // 22 bytes

  // temperature (degrees celcius, 16 bit signed int)
  int16_t temperature;
  // 24 bytes
  
  // accelerometer& magnetometer X Y Z
  uint16_t pitch;
  uint16_t roll;
  uint16_t heading;
  // 30 bytes
  
  // barometric altitude
  int16_t bar_alt;
  // 32 bytes

  // Message Count
  uint16_t message_id;
  // 34 bytes

  // Current Polling Rate
  uint16_t polling_rate;
  // 36 bytes

  // mapped to 0-100% battery capacity
  uint16_t batt_level;
  // 38 bytes
  
  // these are useless and probably take too much space anyway
  //float latitudeDegrees, longitudeDegrees, geoidheight, altitude, speed, angle, magvariation, HDOP;
};

typedef struct statusStruct StatusStruct;

statusStruct buildPacket(const sensors_vec_t &orientation, int16_t &altitude, int16_t &temperature, Adafruit_GPS* GPS, uint16_t &polling_rate) {
  statusStruct ss;
  ss.fix = GPS->fix;
  ss.lat = GPS->lat;
  ss.lon = GPS->lon;
  ss.mag = GPS->mag;
  ss.fixquality = GPS->fixquality;
  ss.satellites = GPS->satellites;
  ss.hour = GPS->hour;
  ss.minute = GPS->minute;
  ss.seconds = GPS->seconds;
  ss.year = GPS->year;
  ss.month = GPS->month;
  ss.day = GPS->day;
  ss.milliseconds = GPS->milliseconds;
  ss.latitude_fixed = GPS->latitude_fixed;
  ss.longitude_fixed = GPS->longitude_fixed;

  ss.temperature = temperature;
  ss.pitch = orientation.pitch;
  ss.roll = orientation.roll;
  ss.heading = orientation.heading;
  ss.bar_alt = altitude;

  ss.message_id = counter;
  ss.polling_rate = polling_rate;

  // get the battery level (value * divider * vRef) note that unit is 1024x higher than the voltage
  float vMeasured = analogRead(VBATPIN) * 2 * 3.3;
  ss.batt_level = (int)(vMeasured);
  

  return ss;


  
  //ss->latitudeDegrees = GPS->latitudeDegrees;
  //ss->longitudeDegrees = GPS->longitudeDegrees;
  //ss->geoidheight = GPS->geoidheight;
  //ss->altitude = GPS->altitude;
  //ss->speed = GPS->speed;
  ///ss->angle = GPS->angle;
  //ss->magvariation = GPS->magvariation;
  //ss->HDOP = GPS->HDOP;
  
}

// Packet to decode from the receiver to set polling rate
struct polling {
  uint32_t message_id;
  
  uint16_t polling_rate;
};
