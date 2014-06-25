#include "Cosa/LCD/Driver/OLED.hh"

OLED::OLED(Board::DigitalPin sdin,
	Board::DigitalPin sclk,
	Font* font) : 
  LCD::Device(),
  m_sdin(sdin, 0),
  m_sclk(sclk, 0),
  m_font(font)
  {  	
  }

void
OLED::set(uint8_t cmd)
{
	asserted(m_)
}