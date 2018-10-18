

// Defined struct for holding all sensor data that the radio will transfer
struct statusStruct {
  boolean fix;
  char lat, lon, mag;
  uint8_t fixquality, satellites, hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  int32_t latitude_fixed, longitude_fixed;
  float latitudeDegrees, longitudeDegrees, geoidheight, altitude, speed, angle, magvariation, HDOP;
};

typedef struct statusStruct StatusStruct;
