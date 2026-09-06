/**
  ******************************************************************************
  * @file    ring.h
  * @brief   UART RX ring buffer interface (USART3 + CH340 serial bridge).
  ******************************************************************************
  */

#ifndef __RING_H
#define __RING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Buffer sizes */
#define CDC_RX_BUFFER_SIZE   512U

/* Ring buffer state (written by ISR via UART_RxByte, read by main via CDC_ProcessRx).
 * Main-loop-only reads, ISR-only writes — no mutex needed. */
extern volatile uint16_t rx_head;   /* next write position (ISR)   */
extern volatile uint16_t rx_tail;   /* next read position (main)   */
extern uint8_t rx_ring[CDC_RX_BUFFER_SIZE];

/* Called from USART3 ISR: push received byte into ring buffer. */
void UART_RxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __RING_H */
