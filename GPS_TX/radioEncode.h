

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

void buildPacket(Adafruit_GPS* GPS, StatusStruct* ss) {
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

  ss->latitudeDegrees = GPS->latitudeDegrees;
  ss->longitudeDegrees = GPS->longitudeDegrees;
  ss->geoidheight = GPS->geoidheight;
  ss->altitude = GPS->altitude;
  ss->speed = GPS->speed;
  ss->angle = GPS->angle;
  ss->magvariation = GPS->magvariation;
  ss->HDOP = GPS->HDOP;
  
}
