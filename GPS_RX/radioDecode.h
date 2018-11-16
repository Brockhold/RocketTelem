

// Defined struct for holding all sensor data that the radio will transfer
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
  
  // accelerometer X Y Z
  uint16_t pitch;
  uint16_t roll;
  uint16_t heading;
  // 30 bytes
  
  // Compass heading
  uint16_t mag_heading;
  // 32 bytes
  
  // barometric altitude (space allowing)
  int16_t bar_alt;
  // 34 bytes;
  
  // these are useless and probably take too much space anyway
  //float latitudeDegrees, longitudeDegrees, geoidheight, altitude, speed, angle, magvariation, HDOP;
};

typedef struct statusStruct StatusStruct;
