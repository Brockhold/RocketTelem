// User interaction options
// Set this to true for additional sensor details
#define DEBUG false
// In headless mode, nothing is written to Serial0 (USB serial)
#define HEADLESS false
// In wait is enabled, the system will pause until Serial0 connects.
#define WAIT true

/************ Radio Setup ***************/
#define RF69_FREQ 915.0     // Label on breakout
#define RFM69_CS      6     // NSS
#define RFM69_INT     11    // DIO 0
#define RFM69_RST     12    // RST
#define PACKET_LEN    24    // Expected packet size

#include <SPI.h>
#include <RH_RF69.h>

unsigned int counter = 0;

#define MINPERIOD 100
