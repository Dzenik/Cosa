/**
 * @file CosaWirelessPing.ino
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2014, Mikael Patel
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
 * @section Description
 * Cosa Wireless interface demo; send-receive messages to and from
 * CosaWirelessPong. Send sequence number to pong and wait for reply
 * with incremented sequence number. Retransit after reply wait and
 * inter-message delay.
 *
 * @section Circuit
 * See Wireless drivers for circuit connections.
 *
 * This file is part of the Arduino Che Cosa project.
 */

#include "Cosa/Trace.hh"
#include "Cosa/IOStream/Driver/UART.hh"
#include "Cosa/Watchdog.hh"
#include "Cosa/RTC.hh"

// Configuration; network and device addresses. 
#define NETWORK 0xC05A
#define PING 0x80
#define PONG 0x81
#define DEVICE PING

// Select Wireless device driver
#define USE_CC1101
// #define USE_NRF24L01P
// #define USE_RFM69
// #define USE_VWI

#if defined(USE_CC1101)
#include "Cosa/Wireless/Driver/CC1101.hh"
CC1101 rf(NETWORK, DEVICE);

#elif defined(USE_NRF24L01P)
#include "Cosa/Wireless/Driver/NRF24L01P.hh"
NRF24L01P rf(NETWORK, DEVICE);

#elif defined(USE_RFM69)
#include "Cosa/Wireless/Driver/RFM69.hh"
RFM69 rf(NETWORK, DEVICE);

#elif defined(USE_VWI)
#include "Cosa/Wireless/Driver/VWI.hh"
#include "Cosa/Wireless/Driver/VWI/Codec/VirtualWireCodec.hh"
VirtualWireCodec codec;
#define BPS 4000
#if defined(BOARD_ATTINY)
VWI rf(NETWORK, DEVICE, BPS, Board::D1, Board::D0, &codec);
#else
VWI rf(NETWORK, DEVICE, BPS, Board::D7, Board::D8, &codec);
#endif
#endif

typedef int16_t ping_t;
static const uint8_t PING_TYPE = 0x80;

void setup()
{
  uart.begin(9600);
  trace.begin(&uart, PSTR("CosaWirelessPing: started"));
  Watchdog::begin();
  RTC::begin();
  ASSERT(rf.begin());
  rf.set_output_power_level(-18);
}

void loop()
{
  // Auto retransmission wait (ms) and counter
  static const uint16_t ARW = 200;
  static uint16_t arc = 0;
  
  // Sequence number
  static ping_t nr = 0;

  // Receive port and source address
  uint8_t port;
  uint8_t src;

  // Send sequence number and receive update
  trace << RTC::millis() << PSTR(":ping:nr=") << nr;
  while (1) {
    rf.send(PONG, PING_TYPE, &nr, sizeof(nr));
    if (rf.recv(src, port, &nr, sizeof(nr), ARW) == sizeof(nr)) break;
    trace << PSTR(",retry(") << ++arc << ')';
  }
  trace << PSTR(",pong:nr=") << nr << endl;
  rf.powerdown();
  sleep(2);
}
