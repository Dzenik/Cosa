/**
 * @file Cosa/IOStream/Driver/UART.hh
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2012, Mikael Patel
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * @section Description
 * Basic UART device handler with internal buffering.
 *
 * This file is part of the Arduino Che Cosa project.
 */

#ifndef __COSA_IOSTREAM_DRIVER_UART_HH__
#define __COSA_IOSTREAM_DRIVER_UART_HH__

#include "Cosa/Types.h"
#include "Cosa/IOStream.hh"
#include "Cosa/IOBuffer.hh"
#include "Cosa/Board.hh"

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define USART_UDRE_vect USART0_UDRE_vect
#define USART_RX_vect USART0_RX_vect 
#endif

extern "C" void USART_UDRE_vect(void) __attribute__ ((signal));
extern "C" void USART_RX_vect(void) __attribute__ ((signal));

class UART : public IOStream::Device {
private:
  volatile uint8_t* const m_sfr;
  IOBuffer* m_ibuf;
  IOBuffer* m_obuf;

  /**
   * Return pointer to UART Control and Status Register A (UCSRnA).
   * @return UCSRAn register pointer.
   */
  volatile uint8_t* UCSRnA() 
  { 
    return (m_sfr); 
  }

  /**
   * Return pointer to UART Control and Status Register B (UCSRnB).
   * @return UCSRnB register pointer.
   */
  volatile uint8_t* UCSRnB() 
  { 
    return (m_sfr + 1); 
  }

  /**
   * Return pointer to UART Control and Status Register C (UCSRnC).
   * @return UCSRnC register pointer.
   */
  volatile uint8_t* UCSRnC() 
  { 
    return (m_sfr + 2); 
  }

  /**
   * Return pointer to UART Baud Rate Register (UBRRn).
   * @return UBRRn register pointer.
   */
  volatile uint16_t* UBRRn() 
  { 
    return ((volatile uint16_t*) (m_sfr + 4)); 
  }

  /**
   * Return pointer to UART I/O Data Register (UDRn).
   * @return UDRn register pointer.
   */
  volatile uint8_t* UDRn() 
  { 
    return (m_sfr + 6); 
  }

  // Interrupt handlers are friends
  friend void USART_UDRE_vect(void);
  friend void USART_RX_vect(void);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  friend void USART1_UDRE_vect(void);
  friend void USART1_RX_vect(void);
  friend void USART2_UDRE_vect(void);
  friend void USART2_RX_vect(void);
  friend void USART3_UDRE_vect(void);
  friend void USART3_RX_vect(void);
#endif

public:
  // Default buffer size for standard UART0
  static const uint8_t BUFFER_MAX = 64;

  // Serial formats; DATA + PARITY + STOP
  enum {
    DATA5 = 0,
    DATA6 = _BV(UCSZ00),
    DATA7 = _BV(UCSZ01),
    DATA8 = _BV(UCSZ01) | _BV(UCSZ00),
    DATA9 = _BV(UCSZ02) | _BV(UCSZ01) | _BV(UCSZ00),
    NO_PARITY = 0,
    EVEN_PARITY = _BV(UPM01),
    ODD_PARITY = _BV(UPM01) | _BV(UPM00),
    STOP1 = 0,
    STOP2 = _BV(USBS0)
  };

  /**
   * Construct serial port handler for given UART.
   * @param[in] port number.
   * @param[in] ibuf input stream buffer.
   * @param[in] obuf output stream buffer.
   */
  UART(uint8_t port, IOBuffer* ibuf, IOBuffer* obuf) : 
    IOStream::Device(),
    m_sfr(Board::UART(port)),
    m_ibuf(ibuf),
    m_obuf(obuf)
  {
  }

  /**
   * @override
   * Number of bytes available in input buffer.
   * @return bytes.
   */
  virtual int available()
  {
    return (m_ibuf->available());
  }

  /**
   * @override
   * Write character to serial port output buffer.
   * Returns character if successful otherwise on error or buffer full
   * returns EOF(-1),
   * @param[in] c character to write.
   * @return character written or EOF(-1).
   */
  virtual int putchar(char c);

  /**
   * @override
   * Read character from serial port input buffer.
   * Returns character if successful otherwise on error or buffer empty
   * returns EOF(-1),
   * @return character or EOF(-1).
   */
  virtual int getchar()
  {
    return (m_ibuf->getchar());
  }

  /**
   * @override
   * Flush internal device buffers. Wait for device to become idle.
   * @return zero(0) or negative error code.
   */
  virtual int flush()
  {
    return (m_ibuf->flush() | m_obuf->flush());
  }

  /**
   * Start UART device driver.
   * @param[in] baudrate serial bitrate (default 9600).
   * @param[in] format serial frame format (default async, 8data, 2stop bit)
   * @return true(1) if successful otherwise false(0)
   */
  bool begin(uint32_t baudrate = 9600, uint8_t format = DATA8 + STOP2);

  /**
   * Stop UART device driver.
   * @return true(1) if successful otherwise false(0)
   */
  bool end();
};

/**
 * Default serial port(0).
 */
extern UART uart;

#endif