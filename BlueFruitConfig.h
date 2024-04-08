#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output

#ifdef Serial    // this makes it not complain on compilation if there's no Serial
  #define BLUEFRUIT_HWSERIAL_NAME      Serial
#endif

#define BLUEFRUIT_UART_MODE_PIN        -1    // Set to -1 if unused

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
