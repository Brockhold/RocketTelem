// User interaction options
// In headless mode, nothing is written to Serial0 (USB serial)
#define HEADLESS false
// In wait is enabled, the system will halt until Serial0 connects.
#define WAIT true
// Default polling rate (should be eventually replaced with dynamic polling rate variable)
#define UPDATE_FREQ 1

// Serial port pinmux for GPS comms, allows for multiple hardware-accelerated serial ports via SERCOM
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3
#define GPSECHO false // echo the GPS data to the Serial console?

// RFM69HCW Radio config
#define RF69_FREQ     915.0                       // radio frequency (MUST MATCH HARDWARE!)
#define RFM69_CS      6                           // chip select on pin D6
#define RFM69_INT     11                          // data ready interrupt on pin D11
#define RFM69_RST     5                           // radio soft-reset on pin D24

// SD card configuration
#define SD_CS 4                                   // chip select on pin D4
