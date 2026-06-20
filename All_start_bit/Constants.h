#ifndef CONSTANTS_H
#define CONSTANTS_H

#define START_BYTE 255
#define BEFORE_PAYLEN_SIZE 3        // 1 START_BYTE + 2 HEADER bytes
#define AFTER_PAYLOAD_SIZE 1
#define BUFFER_SIZE 260

// Configuration options - choose one:
// Option 1: Use bit delay (microseconds)
// #define USE_BIT_DELAY 1
// #define BIT_DELAY_US 1000000

// Option 2: Use frequency (Hz) - uncomment to use frequency instead
#define USE_BIT_DELAY 0
#define FREQUENCY_HZ 100000

#endif
