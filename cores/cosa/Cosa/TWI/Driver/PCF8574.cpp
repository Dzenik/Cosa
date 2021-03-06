/**
 * @file Cosa/TWI/Driver/PCF8574.cpp
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2013-2014, Mikael Patel
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
 * This file is part of the Arduino Che Cosa project.
 */

#include "Cosa/TWI/Driver/PCF8574.hh"

bool
PCF8574::set_data_direction(uint8_t ddr)
{
  m_ddr = ddr;
  m_port |= m_ddr;
  if (!twi.begin(this)) return (false);
  int res = twi.write(&m_port, sizeof(m_port));
  twi.end();
  return (res == sizeof(m_port));
}

uint8_t 
PCF8574::read()
{
  if (!twi.begin(this)) return (0);
  uint8_t res;
  twi.read(&res, sizeof(res));
  twi.end();
  return (res & m_ddr);
}

bool
PCF8574::write(uint8_t pin, uint8_t value)
{
  uint8_t mask = _BV(pin & PIN_MASK);
  if (value)
    m_port |= mask;
  else
    m_port &= ~mask;
  if (!twi.begin(this)) return (false);
  int res = twi.write(&m_port, sizeof(m_port));
  twi.end();
  return (res == sizeof(m_port));
}

bool
PCF8574::write(uint8_t value)
{
  if (!twi.begin(this)) return (false);
  m_port = value | m_ddr;
  int res = twi.write(&m_port, sizeof(m_port));
  twi.end();
  return (res == sizeof(m_port));
}

bool
PCF8574::write(void* buf, size_t size)
{
  if (!twi.begin(this)) return (false);
  uint8_t* bp = (uint8_t*) buf;
  size_t n = size;
  while (n--) *bp++ |= m_ddr;
  int res = twi.write(buf, size);
  twi.end();
  return (res == (int) size);
}
