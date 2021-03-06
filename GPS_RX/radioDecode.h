

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
  
  // accelerometer& magnetometer X Y Z
  int16_t pitch;
  int16_t roll;
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

  //mapped to 0-100% battery capacity
  uint16_t batt_level;
  // 38 bytes

  // sd Card status
  boolean sdStatus;
  // 39 bytes
  
  // these are useless and probably take too much space anyway
  //float latitudeDegrees, longitudeDegrees, geoidheight, altitude, speed, angle, magvariation, HDOP;
};

typedef struct statusStruct StatusStruct;

// Packet to send to the transmitter to set polling rate
struct polling{
  uint32_t message_id;
  
  uint16_t polling_rate;

  char sdCard;

  uint8_t sdCommand;
};
