#ifndef COSA_LCD_DRIVER_OLED_HH
#define COSA_LCD_DRIVER_OLED_HH

#include "Cosa/Board.hh"
#include "Cosa/OutputPin.hh"
#include "Cosa/LCD.hh"
#include "Cosa/Canvas/Font.hh"
#include "Cosa/Canvas/Font/System5x7.hh"

class OLED : public LCD::Device {
public:
  // Display width and height (in pixels)
  static const uint8_t WIDTH = 128;
  static const uint8_t HEIGHT = 64;
  static const uint8_t LINES = 8;

  OLED(Board::DigitalPin sdin = Board::D0,
  		Board::DigitalPin sclk = Board::D1,
  		Font* font = &system5x7);

 /**
   * @override LCD::Device
   * Start interaction with display.
   * @return true(1) if successful otherwise false(0)
   */
  virtual bool begin();

  /**
   * @override LCD::Device
   * Stop sequence of interaction with device.
   * @return true(1) if successful otherwise false(0)
   */
  virtual bool end();

  /**
   * @override LCD::Device
   * Set display contrast (0..63).
   * @param[in] level.
   */
  virtual void display_contrast(uint8_t level);

  /**
   * @override LCD::Device
   * Turn display on. 
   */
  virtual void display_on();

  /**
   * @override LCD::Device
   * Turn display off. 
   */
  virtual void display_off();

  /**
   * @override LCD::Device
   * Display normal mode.
   */
  virtual void display_normal();

  /**
   * @override LCD::Device
   * Display inverse mode. 
   */
  virtual void display_inverse();

  /**
   * @override LCD::Device
   * Clear display and move cursor to home.
   */
  virtual void display_clear();

  /**
   * @override LCD::Device
   * Set cursor to given position.
   * @param[in] x pixel position (0..WIDTH-1).
   * @param[in] y line position (0..LINES-1).
   */
  virtual void set_cursor(uint8_t x, uint8_t y);

  /**
   * Get current text font. 
   * @return font setting.
   */
  Font* get_text_font() { return (m_font); }

  /**
   * Set text font. Returns previous setting.
   * @param[in] font.
   * @return previous font setting.
   */
  Font* set_text_font(Font* font)
  {
    Font* previous = m_font;
    m_font = font;
    return (previous);
  }

  /**
   * Draw icon in the current mode. The icon must be stored in program
   * memory with width, height and data.
   * @param[in] bp
   */
  void draw_icon(const uint8_t* bp);

  /**
   * Draw bitmap in the current mode. 
   * @param[in] bp.
   * @param[in] width.
   * @param[in] height.
   */
  void draw_bitmap(uint8_t* bp, uint8_t width, uint8_t height);

  /**
   * Draw a bar at the current position with the given width.
   * The bar is filled from left to right proportional to the
   * given percent (0..100).
   * @param[in] percent filled from left to right.
   * @param[in] width of bar.
   * @param[in] pattern of filled section of bar.
   */
  void draw_bar(uint8_t percent, uint8_t width, uint8_t pattern = 0x55);

  /**
   * @override IOStream::Device
   * Write character to display. Handles carriage-return-line-feed, back-
   * space, alert, horizontal tab and form-feed. Returns character or EOF 
   * on error.
   * @param[in] c character to write.
   * @return character written or EOF(-1).
   */
  virtual int putchar(char c);

protected:
 /**
   * Write given data to display according to mode.
   * Chip select and/or Command/Data pin asserted.
   * @param[in] data to fill write to device.
   */
  void write(uint8_t data)
  {
    m_si.write(data, m_scl);
  }

  /**
   * Set the given command code.
   * @param[in] cmd command code.
   */
  void set(uint8_t cmd);
  
  /**
   * Set display address for next data block.
   * @param[in] x position (0..WIDTH-1).
   * @param[in] y position (0..LINES-1).
   */
  void set(uint8_t x, uint8_t y);
  
  /**
   * Fill display with given data.
   * @param[in] data to fill with.
   * @param[in] count number of bytes to fill.
   */
  void fill(uint8_t data, uint16_t count);

};

#endif