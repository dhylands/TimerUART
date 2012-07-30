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
*   @file   Timer-UART.cpp
*
*   @brief  Implements the Tx portion of a UART using a timer.
* 
*   This particular code was adapted for Arduino from Timer-UART.c found here:
*   https://github.com/dhylands/projects/blob/master/common/avr/Timer-UART.c
*
*   This started out using the ideas presented in Application Note AVR304
*
*   We need the reload value to fit within 8 bits. The following are some
*   example CPU frequencies divided by baud rates to give you an idea of
*   what baud rates can be used.
*
*   8000000 / 38400 / 8 =  26
*   8000000 /  4800 / 8 = 208
*  16000000 / 38400 / 8 =  52
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include "Timer-UART.h"

#include "WProgram.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// ---- Public Variables ----------------------------------------------------

// ---- Private Constants and Types -----------------------------------------

#if (CFG_TIMER_UART_TIMER == 1)
#define INIT_TIMER \
  do { \
    /* We use the CTC mode (WGM = 4), which starts counting at zero and overflows */ \
    /* when TCNT1 reaches OCR1A. Select prescalar of 8 */ \
    TCCR1A = (0 << WGM11) | (0 << WGM10); \
    TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10); \
  } while (0)
#define TIMER_OCR           OCR1A
#define TIMER_TIMSK         TIMSK1
#define TIMER_OCIE_A        OCIE1A
#define TIMER_TCNT          TCNT1
#define TIMER_TIFR          TIFR1
#define TIMER_OCF_A         OCF1A
#define TIMER_COMPA_vect    TIMER1_COMPA_vect

#elif (CFG_TIMER_UART_TIMER == 2)
#define INIT_TIMER \
  do { \
    /* We use the CTC mode (WGM = 2), which starts counting at zero and overflows */ \
    /* when TCNT2 reaches OCR2A. Select prescalar of 8 */ \
    TCCR2A = (1 << WGM21) | (0 << WGM20); \
    TCCR2B = (0 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20); \
  } while (0)
#define TIMER_OCR           OCR2A
#define TIMER_TIMSK         TIMSK2
#define TIMER_OCIE_A        OCIE2A
#define TIMER_TCNT          TCNT2
#define TIMER_TIFR          TIFR2
#define TIMER_OCF_A         OCF2A
#define TIMER_COMPA_vect    TIMER2_COMPA_vect

#else
#error Only Timer 1 & 2 are supported
#endif


// ---- Private Variables ---------------------------------------------------

TimerUART::TxBuffer_t   TimerUART::sTxBuffer;

uint8_t TimerUART::sTxPin;
bool    TimerUART::sTranslateLFtoCRLF;
uint8_t TimerUART::sTxData;
uint8_t TimerUART::sTxBitCount;

// ---- Private Function Prototypes -----------------------------------------
// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   TimerUART constructor.
*/
TimerUART::TimerUART(uint8_t txPin, bool translateLFtoCRLF)
{
  sTxPin = txPin;
  sTranslateLFtoCRLF = translateLFtoCRLF;
}

//***************************************************************************
/**
*   Initialize the Timer. We need the timer to generate an interrupt for 
*   each bit.
*
*   The reload value represents a single bit time, and we'd like it to 
*   fit in a single byte. Using a smaller prescalar limits the low-end 
*   baud rate, but increases the precision of the bit times.
*
*   8000000 / 38400 / 8 = 26
*/
void TimerUART::init(uint32_t baudRate)
{
#define PRESCALAR   8

    INIT_TIMER;

    // Configure the Tx pin as an output and put it in the marking state

    pinMode(sTxPin, OUTPUT);
    digitalWrite(sTxPin, HIGH);

    TIMER_OCR = (( F_CPU / baudRate ) / PRESCALAR );
}

//***************************************************************************
/**
*   Write a single character on the serial Tx line.
*/
// virtual
void TimerUART::write(uint8_t ch)
{
  if (sTranslateLFtoCRLF) {
    if (ch == '\n') {
        write('\r');
    }
  }

  // Wait for space to become available in the FIFO
  while (sTxBuffer.IsFull()) {
      ;
  }
  sTxBuffer.Push(ch);

  if ((TIMER_TIMSK & (1 << TIMER_OCIE_A)) == 0) {
    // The fact that interrupts are currently disabled means that the
    // character we just put into the FIFO was the first, so we need
    // to re-enable interrupts to kick things off.

    StartByte();

    // We don't stop the timer when interrupts are disabled, so we
    // almost certainly need to clear any indication that an interrupt
    // occured.

    TIMER_TCNT = 0;
    TIMER_TIFR  = (1 << TIMER_OCF_A);

    // Make sure that interrupts are enabled. If this was the first
    // character placed in the FIFO, then when the timer wraps again it
    // will pickup the next character from the FIFO.

    TIMER_TIMSK = (1 << TIMER_OCIE_A);
  }
}

//***************************************************************************
/**
*   Starts sending the byte. Assumes that the timer interrupt has already
*   been enabled.
*/
// static
void TimerUART::StartByte()
{
  // Start things off with the Start bit
  digitalWrite(sTxPin, LOW);

  sTxData = sTxBuffer.Pop();
  sTxBitCount = 9;
}

//***************************************************************************
/**
*   Interrupt handler for Timer2 Compare Match, which happens each time
*   the counter (TCNT2) reaches OCR2A.
*
*   This represents one bit time.
*
*   The basic format of a byte is:
*
*   Line idles HIGH
*   Start bit is LOW
*   Data bits are sent LSB to MSB
*   Stop bit is HIGH
*/
ISR(TIMER_COMPA_vect)
{
  if (TimerUART::sTxBitCount > 1) {
    // Write a data bit.
    digitalWrite(TimerUART::sTxPin, (TimerUART::sTxData & 1) != 0);
    TimerUART::sTxData >>= 1;
  } else {
    // Write the Stop bit
    digitalWrite(TimerUART::sTxPin, HIGH);
  }

  if (TimerUART::sTxBitCount > 0 ) {
    TimerUART::sTxBitCount--;
  } else {
    if (TimerUART::sTxBuffer.IsEmpty()) {
      // Nothing left to transmit, disable the transmit interrupt
      TIMER_TIMSK &= ~(1 << TIMER_OCIE_A);
    } else {
      TimerUART::StartByte();
    }
  }
}  // TIMER_COMPA_vect

