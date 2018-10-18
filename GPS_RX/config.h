// User interaction options
// In headless mode, nothing is written to Serial0 (USB serial)
#define HEADLESS false
// In wait is enabled, the system will halt until Serial0 connects.
#define WAIT true

/************ Radio Setup ***************/
#define RF69_FREQ 915.0
#define RFM69_CS      6
#define RFM69_INT     11
#define RFM69_RST     12
#define PACKET_LEN    24    // Expected packet size
