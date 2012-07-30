/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/
/**
*
*   @file   Timer-UART.h
*
*   @brief  Implements the Tx portion of a UART using a timer.
* 
*   This particular code was adapted for Arduino from Timer-UART.c found here:
*   https://github.com/dhylands/projects/blob/master/common/avr/Timer-UART.c
* 
*   This library was written and tested with arduino-0023.
* 
****************************************************************************/

#if !defined( TIMER_UART_H )
#define TIMER_UART_H                              ///< Include Guard

// ---- Include Files -------------------------------------------------------

#include <inttypes.h>
#include "Print.h"
#include "CBUF.h"

#include <avr/interrupt.h>

//---------------------------------------------------------------------------
//  CFG_TIMER_UART_TX_BUFFER_SIZE   How many bytes to put in the Tx buffer.

#if !defined(CFG_TIMER_UART_TX_BUFFER_SIZE)
#define CFG_TIMER_UART_TX_BUFFER_SIZE   64
#endif

#if (CFG_TIMER_UART_TX_BUFFER_SIZE < 1)
#error CFG_TIMER_UART_TX_BUFFFER_SIZE must be >= 1
#endif

#if ((CFG_TIMER_UART_TX_BUFFER_SIZE & (CFG_TIMER_UART_TX_BUFFER_SIZE - 1)) != 0 )
#   error CFG_TIMER_UART_TX_BUFFER_SIZE isnt a power of 2.
#endif

#if !defined(CFG_TIMER_UART_TIMER)
#define CFG_TIMER_UART_TIMER  1
#endif

#if (CFG_TIMER_UART_TIMER == 1)
#define TIMER_COMPA_vect    TIMER1_COMPA_vect
#elif (CFG_TIMER_UART_TIMER == 2)
#define TIMER_COMPA_vect    TIMER2_COMPA_vect
#else
#error Only Timer 1 & 2 are supported
#endif

//---------------------------------------------------------------------------

extern "C" void TIMER_COMPA_vect(void) __attribute__ ((signal));

class TimerUART : public Print
{
public:
  TimerUART(uint8_t txPin, bool translateLFtoCRLF = true);

  void init(uint32_t baudRate);

  virtual void write(uint8_t ch);

private:

  friend void TIMER_COMPA_vect(void);

  static void StartByte();

#if ( CFG_TIMER_UART_TX_BUFFER_SIZE > 128 )
  typedef uint16_t TxIndex_t;
#else
  typedef uint8_t TxIndex_t;
#endif

  typedef CBUF<TxIndex_t, CFG_TIMER_UART_TX_BUFFER_SIZE, uint8_t>   TxBuffer_t;

  static TxBuffer_t sTxBuffer;
  static  uint8_t   sTxPin;
  static  bool      sTranslateLFtoCRLF;
  static  uint8_t   sTxData;
  static  uint8_t   sTxBitCount;
};

/** @} */

#endif // TIMER_UART_H
