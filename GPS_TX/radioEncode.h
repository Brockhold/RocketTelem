

// Defined struct for holding all sensor data that the radio will transfer
// the maximum size is 64 bytes
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
  
  // these are useless and probably take too much space anyway
  //float latitudeDegrees, longitudeDegrees, geoidheight, altitude, speed, angle, magvariation, HDOP;
};

typedef struct statusStruct StatusStruct;

void buildPacket(sensors_vec_t &orientation, int16_t altitude, int16_t &temperature, Adafruit_GPS* GPS, StatusStruct* ss) {
  ss->fix = GPS->fix;
  ss->lat = GPS->lat;
  ss->lon = GPS->lon;
  ss->mag = GPS->mag;
  ss->fixquality = GPS->fixquality;
  ss->satellites = GPS->satellites;
  ss->hour = GPS->hour;
  ss->minute = GPS->minute;
  ss->seconds = GPS->seconds;
  ss->year = GPS->year;
  ss->month = GPS->month;
  ss->day = GPS->day;
  ss->milliseconds = GPS->milliseconds;
  ss->latitude_fixed = GPS->latitude_fixed;
  ss->longitude_fixed = GPS->longitude_fixed;

  ss->temperature = temperature;
  ss->pitch = orientation.pitch;
  ss->roll = orientation.roll;
  ss->heading = orientation.heading;
  ss->bar_alt = altitude;



  
  //ss->latitudeDegrees = GPS->latitudeDegrees;
  //ss->longitudeDegrees = GPS->longitudeDegrees;
  //ss->geoidheight = GPS->geoidheight;
  //ss->altitude = GPS->altitude;
  //ss->speed = GPS->speed;
  ///ss->angle = GPS->angle;
  //ss->magvariation = GPS->magvariation;
  //ss->HDOP = GPS->HDOP;
  
}
