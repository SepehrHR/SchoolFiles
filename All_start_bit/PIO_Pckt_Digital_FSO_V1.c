#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "Constants.h"
#include "FSO.pio.h"

// Each bit takes precisely 4 PIO cycles
#if USE_BIT_DELAY
    #define PIO_CLOCK_DIVIDER (31.25f * (float)BIT_DELAY_US)
#else
    #define PIO_CLOCK_DIVIDER (125000000.0f / (4.0f * (float)FREQUENCY_HZ))
#endif

const int ledPin = 16;
const int pdPin = 15;

// Packet management
uint8_t packetBuffer[BUFFER_SIZE];
enum PACKET_STATE {
    WaitForStart,
    ReadBeforePayLen,
    ReadPayLen,
    ReadAfterPayLen
};
enum PACKET_STATE packetState = WaitForStart;
int packetPos = 0;
int packetLen = 0;

// Outgoing serial buffer to avoid blocking in packet processing
#define OUTBUF_SIZE 4096
static uint8_t outBuffer[OUTBUF_SIZE];
static volatile uint32_t outHead = 0;
static volatile uint32_t outTail = 0;

static inline bool OutBufferPush(uint8_t b)
{
    uint32_t next = (outHead + 1) & (OUTBUF_SIZE - 1);
    if (next == outTail)
        return false;
    outBuffer[outHead] = b;
    outHead = next;
    return true;
}

static inline bool OutBufferPop(uint8_t *b)
{
    if (outTail == outHead)
        return false;
    *b = outBuffer[outTail];
    outTail = (outTail + 1) & (OUTBUF_SIZE - 1);
    return true;
}

// PIO management
PIO pio = pio0;
uint tx_sm = 0;
uint rx_sm = 1;
uint tx_offset = 0;
uint rx_offset = 0;

void ProcessPacket(uint8_t data);
void SendPacketBufferToSerial();
void ResetPacketReader();
void InitPIO();
void Core1Main();

int main()
{
    stdio_init_all();
    sleep_ms(2000);  // Extended delay for USB enumeration

    printf("\n=== FSO Transceiver Starting ===\n");
    
    InitPIO();
    printf("FSO using optimized PIO mode (Dual-Core)\n");
    
    #if USE_BIT_DELAY
    printf("Bit delay: %d us\n", BIT_DELAY_US);
    #else
    printf("Frequency: %d Hz\n", FREQUENCY_HZ);
    #endif
    printf("PIO Clock Divider: %.2f\n", (float)PIO_CLOCK_DIVIDER);
    
    sleep_ms(500);
    printf("Core 1 launching...\n");

    // Launch Core 1 for packet processing and serial I/O
    multicore_launch_core1(Core1Main);
    
    // Core 0: Handle PIO TX operations
    printf("Core 0 ready - PIO TX active\n");
    
    while(true)
    {
        // Efficiently fill PIO TX FIFO when there's room and data from serial
        if (!pio_sm_is_tx_fifo_full(pio, tx_sm))
        {
            int c = getchar_timeout_us(0);
            if (c != PICO_ERROR_TIMEOUT)
            {
                uint8_t txByte = (uint8_t)c;
                pio_sm_put(pio, tx_sm, ((uint32_t)txByte) << 24);
            }
        }
        
        // Minimal sleep for high responsiveness
        #if USE_BIT_DELAY
            #if BIT_DELAY_US < 20
                tight_loop_contents();  // No sleep for very high speed
            #else
                sleep_us(1);
            #endif
        #endif
    }

    return 0;
}

// ============ PIO Implementation ============

void InitPIO()
{
    // Load TX program
    tx_offset = pio_add_program(pio, &fso_tx_program);
    fso_tx_program_init(pio, tx_sm, tx_offset, ledPin);
    
    // Load RX program
    rx_offset = pio_add_program(pio, &fso_rx_program);
    fso_rx_program_init(pio, rx_sm, rx_offset, pdPin);
    
    // Set clock divider for both state machines
    pio_sm_set_clkdiv(pio, tx_sm, PIO_CLOCK_DIVIDER);
    pio_sm_set_clkdiv(pio, rx_sm, PIO_CLOCK_DIVIDER);
    
    // Enable state machines
    pio_sm_set_enabled(pio, tx_sm, true);
    pio_sm_set_enabled(pio, rx_sm, true);
    
    printf("PIO initialized with clock divider: %.2f\n", (float)PIO_CLOCK_DIVIDER);
}



// ============ Core 1: Serial I/O & Packet Processing ============

void Core1Main()
{
    sleep_ms(1500);  // Wait for Core 0 to finish initialization
    
    printf("Core 1 ready - Serial I/O active\n");
    
    while(true)
    {
        // Efficiently drain any incoming RX data from PIO
        while (!pio_sm_is_rx_fifo_empty(pio, rx_sm))
        {
            uint32_t received = pio_sm_get(pio, rx_sm);
            uint8_t rxByte = (uint8_t)(received & 0xFF);
            ProcessPacket(rxByte);
        }

        // Flush a limited amount of outgoing serial data so RX processing remains fast
        const int max_flush = 64;
        int flushed = 0;
        uint8_t b;
        while (flushed < max_flush && OutBufferPop(&b))
        {
            putchar(b);
            flushed++;
        }
        
        // Very light sleep to allow other tasks
        sleep_us(2);
    }
}

// ============ Packet Processing ============

void ProcessPacket(uint8_t data)
{
    switch (packetState)
    {
    case WaitForStart:
        if (data == START_BYTE)
        {
            packetPos = 0;
            packetLen = 0;
            packetBuffer[packetPos++] = data;
            packetState = ReadBeforePayLen;
        }
        break;

    case ReadBeforePayLen:
        packetBuffer[packetPos++] = data;
        if (packetPos >= BEFORE_PAYLEN_SIZE)
        {
            packetState = ReadPayLen;
        }
        break;

    case ReadPayLen:
        packetBuffer[packetPos++] = data;
        packetLen = BEFORE_PAYLEN_SIZE + 1 + (int)data + AFTER_PAYLOAD_SIZE;
        if (packetLen > BUFFER_SIZE)
        {
            ResetPacketReader();
            break;
        }
        packetState = ReadAfterPayLen;
        break;

    case ReadAfterPayLen:
        packetBuffer[packetPos++] = data;
        if (packetPos >= packetLen)
        {
            SendPacketBufferToSerial();
            ResetPacketReader();
        }
        break;
    }
}

inline void SendPacketBufferToSerial()
{
    for (int i = 0; i < packetLen; i++)
    {
        if (!OutBufferPush(packetBuffer[i]))
        {
            break;
        }
    }
}

inline void ResetPacketReader()
{
    packetState = WaitForStart;
    packetPos = 0;
    packetLen = 0;
}