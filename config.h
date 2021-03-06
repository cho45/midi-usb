#ifndef __CONFIG_H__
#define __CONFIG_H__

Serial serial(UART_TX, UART_RX);

#define DEBUG 1

// Generic DEBUG_PRINTF must not used at any interrupt
#if DEBUG
#define DEBUG_PRINTF(...) serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

#endif
