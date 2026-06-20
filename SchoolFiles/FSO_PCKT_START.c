#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pio_instructions.h"
#include "Constants.h"
#include "FSO.pio.h"

const int ledPin = 16;
const int pdPin = 15;

// RX Packet management
uint8_t rxPacketBuffer[BUFFER_SIZE];
enum PACKET_STATE { WaitForStart, ReadBeforePayLen, ReadPayLen, ReadAfterPayLen };
volatile enum PACKET_STATE rxPacketState = WaitForStart;
int rxPacketPos = 0;
int rxPacketLen = 0;

// TX Packet management
uint8_t txPacketBuffer[BUFFER_SIZE];
enum PACKET_STATE txPacketState = WaitForStart;
int txPacketPos = 0;
int txPacketLen = 0;

// Outgoing serial buffer to avoid blocking in packet processing
#define OUTBUF_SIZE 4096
static uint8_t outBuffer[OUTBUF_SIZE];
static volatile uint32_t outHead = 0;
static volatile uint32_t outTail = 0;

static inline bool OutBufferPush(uint8_t b) {
    uint32_t next = (outHead + 1) & (OUTBUF_SIZE - 1);
    if (next == outTail) return false;
    outBuffer[outHead] = b;
    outHead = next;
    return true;
}

static inline bool OutBufferPop(uint8_t *b) {
    if (outTail == outHead) return false;
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
uint32_t current_freq_hz = 0;

// TX: number of PIO cycles per bit. Both TX and RX must use the same value.
// The PIO programs are hardcoded to 4 cycles per bit.
#define CYCLES_PER_BIT 4

// Command management
enum Commands { Data = 0x00, Frequency = 0x01 };
enum CmdState { ReadCommand, ReadLength, ReadData, ReadFrequency };
enum CmdState cmd_state = ReadCommand;
uint8_t dataLenHighLow[] = {0, 0};
uint16_t dataLen = 0;
int dataLenIndex = 0;
uint32_t dataIndex = 0;

uint8_t frequencyPacked[] = {0, 0, 0, 0};
int frequencyIndex = 0;

// Function prototypes
void __not_in_flash_func(ProcessRxByte)(uint8_t data);
void __not_in_flash_func(Core1Main)();
void ProcessTxByte(uint8_t data);
void __not_in_flash_func(SendRxPacketToSerial)();
void __not_in_flash_func(SendTxPacketToPIO)();
void __not_in_flash_func(ResetRxReader)();
void __not_in_flash_func(ResetRXPio)();
void ResetTxReader();
void SetFrequency(uint32_t freq_hz);
void ManageCommand(uint8_t data);
void InitPIO();

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== FSO Transceiver Starting ===\n");
    InitPIO();

    sleep_ms(500);
    printf("Core 1 launching...\n");

    multicore_launch_core1(Core1Main);
    printf("Core 0 ready - PIO TX active\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            ManageCommand((uint8_t)c);
        }
    }
    return 0;
}

void InitPIO() {
    tx_offset = pio_add_program(pio, &fso_tx_program);
    fso_tx_program_init(pio, tx_sm, tx_offset, ledPin);

    rx_offset = pio_add_program(pio, &fso_rx_program);
    fso_rx_program_init(pio, rx_sm, rx_offset, pdPin);

    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    float target_freq_hz;

#if USE_BIT_DELAY
    target_freq_hz = 1000000.0f / (float)BIT_DELAY_US;
#else
    target_freq_hz = (float)FREQUENCY_HZ;
#endif

    float pio_clock_divider = (float)sys_clk_hz / ((float)CYCLES_PER_BIT * target_freq_hz);
    if (pio_clock_divider < 1.0f) pio_clock_divider = 1.0f;

    current_freq_hz = (uint32_t)(sys_clk_hz / ((float)CYCLES_PER_BIT * pio_clock_divider));

    pio_sm_set_clkdiv(pio, tx_sm, pio_clock_divider);
    pio_sm_set_clkdiv(pio, rx_sm, pio_clock_divider);

    pio_sm_set_enabled(pio, tx_sm, true);
    pio_sm_set_enabled(pio, rx_sm, true);

    printf("Requested Freq : %.0f Hz\n", target_freq_hz);
    printf("Actual Freq    : %u Hz\n", current_freq_hz);
    printf("System Clock   : %u MHz | Divider: %.4f\n",
           sys_clk_hz / 1000000, pio_clock_divider);
}

void __not_in_flash_func(Core1Main)() {
    sleep_ms(1500);
    printf("Core 1 ready - RAM polling active\n");

    while (true) {
        while (!pio_sm_is_rx_fifo_empty(pio, rx_sm)) {
            uint32_t received = pio_sm_get(pio, rx_sm);
            ProcessRxByte((uint8_t)(received & 0xFF));
        }

        // Batch USB push; only when idle (not mid-packet)
        if (rxPacketState == WaitForStart) {
            uint8_t buffer[64];
            int count = 0;
            while (count < 64 && OutBufferPop(&buffer[count])) {
                count++;
                if (!pio_sm_is_rx_fifo_empty(pio, rx_sm)) break; // Prioritize PIO
            }
            if (count > 0) {
                for (int i = 0; i < count; i++) {
                    putchar_raw(buffer[i]);
                }
                stdio_flush();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ResetRXPio
// ---------------------------------------------------------------------------
// Restarts the RX state machine to the 'wait 0 pin 0' entry point WITHOUT
// toggling pio_sm_set_enabled. Toggling enabled takes ~microseconds of C
// overhead which at 8–31 MHz corresponds to many bit periods and causes the
// receiver to miss the next packet's start bit.
//
// pio_sm_restart clears the clock divider phase and all internal state, then
// we immediately jump to the program start so it waits for idle LOW.
// ---------------------------------------------------------------------------
void __not_in_flash_func(ResetRXPio)() {
    pio_sm_clear_fifos(pio, rx_sm);
    pio_sm_restart(pio, rx_sm);
    pio_sm_exec(pio, rx_sm, pio_encode_jmp(rx_offset)); // Jump to 'wait 0 pin 0'
}

void __not_in_flash_func(ProcessRxByte)(uint8_t data) {
    switch (rxPacketState) {
    case WaitForStart:
        if (data == START_BYTE) {
            rxPacketPos = 0;
            rxPacketLen = 0;
            rxPacketBuffer[rxPacketPos++] = data;
            rxPacketState = ReadBeforePayLen;
        }
        // No ResetRXPio here: PIO was already reset after the previous packet
        // completed (or at startup). Resetting again on every stray byte would
        // cost many bit-periods of blind time at high frequencies.
        break;

    case ReadBeforePayLen:
        rxPacketBuffer[rxPacketPos++] = data;
        if (rxPacketPos >= BEFORE_PAYLEN_SIZE) {
            rxPacketState = ReadPayLen;
        }
        break;

    case ReadPayLen:
        rxPacketBuffer[rxPacketPos++] = data;
        rxPacketLen = BEFORE_PAYLEN_SIZE + 1 + (int)data + AFTER_PAYLOAD_SIZE;
        if (rxPacketLen > BUFFER_SIZE) {
            ResetRxReader();
            ResetRXPio(); // Malformed packet - hard reset to re-sync
            break;
        }
        rxPacketState = ReadAfterPayLen;
        break;

    case ReadAfterPayLen:
        rxPacketBuffer[rxPacketPos++] = data;
        if (rxPacketPos >= rxPacketLen) {
            SendRxPacketToSerial();
            ResetRxReader();
            ResetRXPio(); // Must reset: wrap loop has no exit, so PIO never
                          // returns to 'wait 0' on its own between packets.
        }
        break;
    }
}

inline void __not_in_flash_func(SendRxPacketToSerial)() {
    for (int i = 0; i < rxPacketLen; i++) {
        if (!OutBufferPush(rxPacketBuffer[i])) break;
    }
}

inline void __not_in_flash_func(ResetRxReader)() {
    rxPacketState = WaitForStart;
    rxPacketPos = 0;
    rxPacketLen = 0;
}

void ProcessTxByte(uint8_t data) {
    switch (txPacketState) {
    case WaitForStart:
        if (data == START_BYTE) {
            txPacketPos = 0;
            txPacketLen = 0;
            txPacketBuffer[txPacketPos++] = data;
            txPacketState = ReadBeforePayLen;
        }
        break;

    case ReadBeforePayLen:
        txPacketBuffer[txPacketPos++] = data;
        if (txPacketPos >= BEFORE_PAYLEN_SIZE) {
            txPacketState = ReadPayLen;
        }
        break;

    case ReadPayLen:
        txPacketBuffer[txPacketPos++] = data;
        txPacketLen = BEFORE_PAYLEN_SIZE + 1 + (int)data + AFTER_PAYLOAD_SIZE;
        if (txPacketLen > BUFFER_SIZE) {
            ResetTxReader();
        } else {
            txPacketState = ReadAfterPayLen;
        }
        break;

    case ReadAfterPayLen:
        txPacketBuffer[txPacketPos++] = data;
        if (txPacketPos >= txPacketLen) {
            SendTxPacketToPIO();
            ResetTxReader();
        }
        break;
    }
}

void __not_in_flash_func(SendTxPacketToPIO)() {
    uint32_t totalBits = txPacketLen * 8;
    pio_sm_put_blocking(pio, tx_sm, totalBits - 1);

    uint32_t word = 0;
    int shift = 24;
    for (int i = 0; i < txPacketLen; i++) {
        word |= ((uint32_t)txPacketBuffer[i]) << shift;
        if (shift == 0) {
            pio_sm_put_blocking(pio, tx_sm, word);
            word = 0;
            shift = 24;
        } else {
            shift -= 8;
        }
    }
    if (shift != 24) {
        pio_sm_put_blocking(pio, tx_sm, word);
    }

    // Wait for the TX FIFO to drain to empty
    while (!pio_sm_is_tx_fifo_empty(pio, tx_sm)) {
        tight_loop_contents();
    }

    // Wait for the PIO shift register to finish clocking out its remaining bits.
    // The shift register holds up to 32 bits plus the 8-bit start-bit overhead,
    // so worst case is 40 bits still in flight when the FIFO goes empty.
    // Use ceiling division to avoid returning early at high frequencies.
    uint32_t wait_us = ((40u * 1000000u) + current_freq_hz - 1u) / current_freq_hz;
    busy_wait_us(wait_us + 1); // +1 µs safety margin
}

inline void ResetTxReader() {
    txPacketState = WaitForStart;
    txPacketPos = 0;
    txPacketLen = 0;
}

// ---------------------------------------------------------------------------
// SetFrequency
// ---------------------------------------------------------------------------
// Changes the PIO clock divider on both state machines without toggling
// pio_sm_set_enabled. Disabling and re-enabling the SM costs hundreds of
// nanoseconds to microseconds of overhead. Setting the divider while running
// takes effect within one PIO clock cycle, which is safe because:
//   - TX is idle (SendTxPacketToPIO already waited for it to finish)
//   - RX is reset immediately after via ResetRXPio so any partial bit
//     captured under the old divider is discarded
// ---------------------------------------------------------------------------
void SetFrequency(uint32_t freq_hz) {
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    float pio_clock_divider = (float)sys_clk_hz / ((float)CYCLES_PER_BIT * (float)freq_hz);

    if (pio_clock_divider < 1.0f) {
        pio_clock_divider = 1.0f;
    }

    current_freq_hz = (uint32_t)(sys_clk_hz / ((float)CYCLES_PER_BIT * pio_clock_divider));

    pio_sm_set_clkdiv(pio, tx_sm, pio_clock_divider);
    pio_sm_set_clkdiv(pio, rx_sm, pio_clock_divider);

    // Reset the RX state machine so any partial state from the old clock
    // rate is discarded and it resynchronizes from idle.
    ResetRXPio();
}

void ManageCommand(uint8_t data) {
    switch (cmd_state) {
    case ReadCommand:
        switch (data) {
        case Data:
            cmd_state = ReadLength;
            dataLen = 0;
            dataIndex = 0;
            dataLenIndex = 0;
            break;
        case Frequency:
            cmd_state = ReadFrequency;
            frequencyIndex = 0;
            break;
        }
        break;
    case ReadLength:
        dataLenHighLow[dataLenIndex++] = data;
        if (dataLenIndex > 1) {
            cmd_state = ReadData;
            dataLen = ((uint16_t)dataLenHighLow[0] << 8) | (uint16_t)dataLenHighLow[1];
            if (dataLen == 0) cmd_state = ReadCommand;
        }
        break;
    case ReadData:
        ProcessTxByte(data);
        dataIndex++;
        if (dataIndex >= dataLen) {
            cmd_state = ReadCommand;
        }
        break;
    case ReadFrequency:
        frequencyPacked[frequencyIndex++] = data;
        if (frequencyIndex > 3) {
            uint32_t frequency = ((uint32_t)frequencyPacked[0] << 24) |
                                 ((uint32_t)frequencyPacked[1] << 16) |
                                 ((uint32_t)frequencyPacked[2] << 8) |
                                 frequencyPacked[3];
            SetFrequency(frequency);
            frequencyIndex = 0;
            cmd_state = ReadCommand;
        }
        break;
    }
}
