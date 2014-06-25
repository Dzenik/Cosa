/**
 * @file Cosa/Wireless/Driver/RFM22.hh
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
 * This file is part of the Arduino Che Cosa project.
 */

#ifndef COSA_WIRELESS_DRIVER_RFM22_HH
#define COSA_WIRELESS_DRIVER_RFM22_HH

#include "Cosa/SPI.hh"
#include "Cosa/OutputPin.hh"
#include "Cosa/ExternalInterrupt.hh"
#include "Cosa/Wireless.hh"
#if !defined(BOARD_ATTINYX5)

/**
 * Cosa Device Driver for RFM69W/HW, Low-Power Sub-1 GHz RF Transceiver. 
 * Note that this device requires data in big endian order. 
 *
 * @section Circuit
 * This is the pin-out for the RFM69W/HW module.
 * @code
 *                         RFM69W/HW
 *                       +------------+
 * (RST)---------------1-|RESET     NC|-16
 * (D2/EXT0)-----------2-|DIO0     NSS|-15---------------(D10)
 *                     3-|DIO1    MOSI|-14---------------(D11/MOSI)
 *                     4-|DIO2    MISO|-13---------------(D12/MISO)
 *                     5-|DIO3     SCK|-12---------------(D13/SCK)     V
 *                     6-|DIO4     GND|-11---------------(GND)         |
 *                     7-|DIO4     ANT|-10-----------------------------+
 * (3V3)---------------8-|VCC      GND|-9----------------(GND)
 *                       +------------+
 * @endcode
 *
 * @section References
 * 1. Product datasheet, RFM69W ISM Transceiver Module V1.3, 
 * http://www.hoperf.com/rf/fsk_module/RFM69W.htm
 * 2. Product datasheet, RFM69HW ISM Transceiver Module V1.3, 
 * http://www.hoperf.com/rf/fsk_module/RFM69HW.htm
 */
class RFM22 : private SPI::Driver, public Wireless::Driver {
public:
  /**
   * Maximum size of frame header is dest(1), src(1), and port(1).
   */
  static const size_t HEADER_MAX = 3;

  /**
   * Maximum size of payload. The device allows 66 bytes payload. 
   * Adjust for frame header.
   */
  static const size_t PAYLOAD_MAX = 66 - HEADER_MAX;
  
  /**
   * Construct RFM22 device driver with given network and device
   * address. Connected to SPI bus and given chip select pin. Default
   * pins are Arduino Nano IO Shield for RFM22 module are D10 chip
   * select (RFM22:NSS) and D2/EXT0 external interrupt pin (RFM22:DIO0).
   * @param[in] net network address.
   * @param[in] dev device address.
   * @param[in] csn chip select pin (Default D2/D10/D53).
   * @param[in] irq interrupt pin (Default EXT0).
   */
#if defined(BOARD_ATTINYX4)
  RFM22(uint16_t net, uint8_t dev, 
    Board::DigitalPin csn = Board::D2,
    Board::ExternalInterruptPin irq = Board::EXT0);
#elif defined(BOARD_ATMEGA2560)
  RFM22(uint16_t net, uint8_t dev, 
    Board::DigitalPin csn = Board::D53,
    Board::ExternalInterruptPin irq = Board::EXT4);
#else
  RFM22(uint16_t net, uint8_t dev, 
    Board::DigitalPin csn = Board::D10,
    Board::ExternalInterruptPin irq = Board::EXT0);
#endif

  /**
   * @override Wireless::Driver
   * Start and configure RFM22 device driver. The configuration must
   * set DIO0 to assert on received message. This device pin is
   * assumed to be connected the device driver interrupt pin (EXTn).
   * Return true(1) if successful othewise false(0).
   * @param[in] config configuration vector (default NULL)
   */
  virtual bool begin(const void* config = NULL);

  /**
   * @override Wireless::Driver
   * Shut down the device driver. Return true(1) if successful
   * otherwise false(0).
   * @return bool
   */
  virtual bool end();
    
  /**
   * @override Wireless::Driver
   * Send message in given null terminated io vector. Returns number
   * of bytes sent. Returns error code(-1) if number of bytes is
   * greater than PAYLOAD_MAX. Return error code(-2) if fails to set
   * transmit mode and/or packet is available to receive.
   * @param[in] dest destination network address.
   * @param[in] port device port (or message type).
   * @param[in] vec null termianted io vector.
   * @return number of bytes send or negative error code.
   */
  virtual int send(uint8_t dest, uint8_t port, const iovec_t* vec);

  /**
   * @override Wireless::Driver
   * Send message in given buffer, with given number of bytes. Returns
   * number of bytes sent. Returns error code(-1) if number of bytes
   * is greater than PAYLOAD_MAX. Return error code(-2) if fails to
   * set transmit mode.  
   * @param[in] dest destination network address.
   * @param[in] port device port (or message type).
   * @param[in] buf buffer to transmit.
   * @param[in] len number of bytes in buffer.
   * @return number of bytes send or negative error code.
   */
  virtual int send(uint8_t dest, uint8_t port, const void* buf, size_t len);

  /**
   * @override Wireless::Driver
   * Receive message and store into given buffer with given maximum
   * length. The source network address is returned in the parameter src.
   * Returns error code(-2) if no message is available and/or a
   * timeout occured. Returns error code(-1) if the buffer size if to
   * small for incoming message or if the receiver fifo has overflowed. 
   * Otherwise the actual number of received bytes is returned
   * @param[out] src source network address.
   * @param[out] port device port (or message type).
   * @param[in] buf buffer to store incoming message.
   * @param[in] len maximum number of bytes to receive.
   * @param[in] ms maximum time out period.
   * @return number of bytes received or negative error code.
   */
  virtual int recv(uint8_t& src, uint8_t& port, void* buf, size_t len, 
           uint32_t ms = 0L);

  /**
   * @override Wireless::Driver
   * Set device in power down mode. 
   */
  virtual void powerdown();

  /**
   * @override Wireless::Driver
   * Set device in wakeup on radio mode. 
   */
  virtual void wakeup_on_radio();
  
  /**
   * @override Wireless::Driver
   * Set output power level [-18..13] dBm.
   * @param[in] dBm.
   */
  virtual void set_output_power_level(int8_t dBm);

  /**
   * @override Wireless::Driver
   * Return estimated input power level (dBm) from latest successful
   * message received. 
   */
  virtual int get_input_power_level();

  /**
   * Sample internal digital thermometer and return in centigrade
   * Celsius.
   * @return temperature.
   */
  int get_temperature();

  /**
   * Recalibrate internal RC oscillator when device is used in an
   * environment with high temperature variation.
   */
  void recalibrate();

private:
  /**
   * Configuration and Status Registers (Table 23, pp. 60).
   */
  enum Reg {
    DEVICE_TYPE = 0x00,     //!< FIFO read/write access (66 byte).
    VERSION_CODE = 0x01,        //!< Operating modes of the transceiver.
    DEVICE_STATUS = 0x02,       //!< Data operation mode and modulation.
    INTERRUPT_STATUS1 = 0x03,       //!< Bit Rate setting (16-bit).
    INTERRUPT_STATUS2 = 0x04,       //!< Bit Rate setting (LSB).
    INTERRUPT_ENABLE1 = 0x05,           //!< Frequency Deviation setting (16-bit).
    INTERRUPT_ENABLE2 = 0x06,           //!< Frequency Deviation setting (LSB).
    OPERATING_MODE1 = 0x07,         //!< RF Carrier Frequency (24-bit).
    OPERATING_MODE2 = 0x08,     //!< RF Carrier Frequency (MID).
    OSCILLATOR_LOAD_CAPACITANCE = 0x09,     //!< RF Carrier Frequency (LSB).
    UC_OUTPUT_CLOCK = 0x0A,         //!< RC Oscillators Settings.
    GPIO_CONFIGURATION0 = 0x0B,     //!< AFC control in low modulation.
    GPIO_CONFIGURATION1 = 0x0C,
    GPIO_CONFIGURATION2 = 0x0D,     //!< Listen Mode settings.
    IO_PORT_CONFIGURATION = 0x0E,       //!< Listen Mode Idle duration.
    ADC_CONFIGURATION = 0x0F,       //!< Listen Mode Rx duration.
    ADC_SENSOR_AMP_OFFSET = 0x10,       //!< Version.
    ADC_VALUE = 0x11,           //!< PA selection and Output Power control.
    TEMPERATURE_SENSOR_CALIBRATION = 0x12,          //!< Control of the PA ramp time in FSK mode.
    TEMPERATURE_VALUE_OFFSET = 0x13,            //!< Over Current Protection control.
    WAKEUP_TIMER_PERIOD1 = 0x14,
    WAKEUP_TIMER_PERIOD2 = 0x15,
    WAKEUP_TIMER_PERIOD3 = 0x16,
    WAKEUP_TIMER_VALUE1 = 0x17,
    WAKEUP_TIMER_VALUE2 = 0x18,         //!< LNA settings.
    LDC_MODE_DURATION = 0x19,       //!< Channel Filter BW Control.
    LOW_BATTERY_DETECTOR_THRESHOLD = 0x1A,      //!< Channel Filter BW Control durint AFC.
    BATTERY_VOLTAGE_LEVEL = 0x1B,            //!< OOK demodulator control in peak mode.
    IF_FILTER_BANDWIDTH = 0x1C,             //!< OOK demodulator average threshold control.
    AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,         //!< OOK demodulator fixed threshold control.
    AFC_TIMING_CONTROL = 0x1E,      //!< AFC and FEI control and status.
    CLOCK_RECOVERY_GEARSHIFT_OVERRIDE = 0x1F,           //!< Frequency correction of the AFC (16-bit).
    CLOCK_RECOVERY_OVERSAMPLING_RATE = 0x20,        //!< Frequency correction of the AFC (LSB).
    CLOCK_RECOVERY_OFFSET2 = 0x21,          //!< Calculated frequency error (16-bit).
    CLOCK_RECOVERY_OFFSET1 = 0x22,      //!< Calculated frequency error (LSB).
    CLOCK_RECOVERY_OFFSET0 = 0x23,      //!< RSSI-related settings.
    CLOCK_RECOVERY_TIMING_LOOP_GAIN1 = 0x24,        //!< RSSI value in dBm.
    CLOCK_RECOVERY_TIMING_LOOP_GAIN0 = 0x25,    //!< Mapping of pins DIO0 to DIO3.
    RSSI = 0x26,    //!< Mapping of pins DIO4 to DIO5, CLKOUT frequency.
    RSSI_THRESHOLD = 0x27,      //!< PLL Lock state, Timeout, RSSI Threshold...
    ANTENNA_DIVERSITY1 = 0x28,          //!< FIFO handling flags...
    ANTENNA_DIVERSITY2 = 0x29,      //!< RSSI Threshold control.
    AFC_LIMITER = 0x2A,     //!< Time from Rx request to RSSI detection.
    AFC_CORRECTION_READ = 0x2B,     //!< Time from RSSI detection and PayloadReady.
    OOK_COUNTER_VALUE_1 = 0x2C,     //!< Preamble length (16-bit).
    OOK_COUNTER_VALUE_2 = 0x2D, //!< Preamble length (LSB).
    SLICER_PEAK_HOLD = 0x2E,        //!< Sync Word Recognition control.
    DATA_ACCESS_CONTROL = 0x30,     //!< Byte 2 of Sync Word.
    EZMAC_STATUS = 0x31,        //!< Byte 3 of Sync Word.
    HEADER_CONTROL1 = 0x32,     //!< Byte 4 of Sync Word.
    HEADER_CONTROL2 = 0x33,     //!< Byte 5 of Sync Word.
    PREAMBLE_LENGTH = 0x34,     //!< Byte 6 of Sync Word.
    PREAMBLE_DETECTION_CONTROL1 = 0x35,     //!< Byte 7 of Sync Word.
    SYNC_WORD3 = 0x36,      //!< Byte 8 of Sync Word.
    SYNC_WORD2 = 0x37,  //!< Packet mode settings.
    SYNC_WORD1 = 0x38,  //!< Payload length setting.
    SYNC_WORD0 = 0x39,      //!< Node address.
    TRANSMIT_HEADER3 = 0x3A,    //!< Broadcast address.
    TRANSMIT_HEADER2 = 0x3B,        //!< Auto modes settings.
    TRANSMIT_HEADER1 = 0x3C,    //!< Fifo threshold, Tx start condition.
    TRANSMIT_HEADER0 = 0x3D,    //!< Package mode settings.
    PACKET_LENGTH = 0x3E,       //!< Cypher key (16 bytes).
    CHECK_HEADER3 = 0x3F,
    CHECK_HEADER2 = 0x40,
    CHECK_HEADER1 = 0x41,
    CHECK_HEADER0 = 0x42,
    HEADER_ENABLE3 = 0x43,
    HEADER_ENABLE2 = 0x44,
    HEADER_ENABLE1 = 0x45,
    HEADER_ENABLE0 = 0x46,
    RECEIVED_HEADER3 = 0x47,
    RECEIVED_HEADER2 = 0x48,
    RECEIVED_HEADER1 = 0x49,
    RECEIVED_HEADER0 = 0x4A,
    RECEIVED_PACKET_LENGTH = 0x4B,
    ANALOG_TEST_BUS_SELECT = 0x50,
    DIGITAL_TEST_BUS_SELECT = 0x51,
    TX_RAMP_CONTROL = 0x52,
    PLL_TUNE_TIME = 0x53,
    CALIBRATION_CONTROL = 0x55,
    MODEM_TEST = 0x56,
    CHARGE_PUMP_TEST = 0x57,
    CHARGE_PUMP_CURRENT_TRIMMING = 0x58,        //!< Sensitivity boost.
    DIVIDER_CURRENT_TRIMMING = 0x59,
    VCO_CURRENT_TRIMMING = 0x5A,        //!< High Power PA settings.
    VCO_CALIBRATION = 0x5B,
    SYNTHESIZER_TEST = 0x5C,        //!< High Power PA settings.
    BLOCK_ENABLE_OVERRIDE1 = 0x5D,
    BLOCK_ENABLE_OVERRIDE2 = 0x5E,
    BLOCK_ENABLE_OVERRIDE3 = 0x5F,
    CHANNEL_FILTER_COEFFICIENT_ADDRESS = 0x60,
    CHANNEL_FILTER_COEFFICIENT_VALUE = 0x61,
    CRYSTAL_OSCILLATOR_POR_CONTROL = 0x62,
    RC_OSCILLATOR_COARSE_CALIBRATION = 0x63,
    RC_OSCILLATOR_FINE_CALIBRATION = 0x64,
    LDO_CONTROL_OVERRIDE = 0x65,
    LDO_LEVEL_SETTINGS = 0x66,
    DELTA_SIGMA_ADC_TUNING1 = 0x67,
    DELTA_SIGMA_ADC_TUNING2 = 0x68,
    AGC_OVERRIDE1 = 0x69,
    AGC_OVERRIDE2 = 0x6A,
    GFSK_FIR_FILTER_COEFFICIENT_ADDRESS = 0x6B,
    GFSK_FIR_FILTER_COEFFICIENT_VALUE = 0x6C,
    TX_POWER = 0x6D,
    TX_DATA_RATE1 = 0x6E,
    TX_DATA_RATE0 = 0x6F,       //!< Fading Margin Improvement.
    MODULATION_CONTROL1 = 0x70,
    MODULATION_CONTROL2 = 0x71,     //!< AFC offset for low modulation index AFC.
    FREQUENCY_DEVIATION = 0x72,
    FREQUENCY_OFFSET1 = 0x73,
    FREQUENCY_OFFSET2 = 0x74,
    FREQUENCY_BAND_SELECT = 0x75,
    NOMINAL_CARRIER_FREQUENCY1 = 0x76,
    NOMINAL_CARRIER_FREQUENCY0 = 0x77,
    FREQUENCY_HOPPING_CHANNEL_SELECT = 0x79,
    FREQUENCY_HOPPING_STEP_SIZE = 0x7A,
    TX_FIFO_CONTROL1 = 0x7C,
    TX_FIFO_CONTROL2 = 0x7D,
    RX_FIFO_CONTROL = 0x7E,
    FIFO_ACCESS = 0x7F
 } __attribute__((packed));
  
  /**
   * Transaction header (figure 25, pp. 44). Register address and 
   * read/write flag in most significant bit.
   */
  enum {
    REG_READ = 0x00,        //!< Read register.
    REG_WRITE = 0x80,       //!< Write register.
    REG_MASK = 0x7F     //!< Mask register.
  } __attribute__((packed));
  
  /**
   * Read single register value.
   * @param[in] reg register address.
   * @return value
   */
  uint8_t read(Reg reg)
  {
    spi.begin(this);
    spi.transfer(REG_READ | reg);
    uint8_t res = spi.transfer(0);
    spi.end();
    return (res);
  }

  /**
   * Read multiple register or fifo values into given buffer. 
   * @param[in] reg start register or fifo address.
   * @param[in] buf buffer to store register values.
   * @param[in] count size of buffer and number of registers to read.
   */
  void read(Reg reg, void* buf, size_t count)
  {
    spi.begin(this);
    spi.transfer(REG_READ | reg);
    spi.read(buf, count);
    spi.end();
  }

  /**
   * Write single register value.
   * @param[in] reg register address.
   * @param[in] value to write to register.
   */
  void write(Reg reg, uint8_t value)
  {
    spi.begin(this);
    spi.transfer(REG_WRITE | reg);
    spi.transfer(value);
    spi.end();
  }

  /**
   * Write multiple register values or fifo from given buffer. 
   * @param[in] reg start register address.
   * @param[in] buf buffer with new register values.
   * @param[in] count size of buffer and number of registers to read.
   */
  void write(Reg reg, const void* buf, size_t count)
  {
    spi.begin(this);
    spi.transfer(REG_WRITE | reg);
    spi.write(buf, count);
    spi.end();
  }

  /**
   * Write multiple register values from given buffer in program memory.
   * @param[in] reg start register address.
   * @param[in] buf buffer in program memory with new register values.
   * @param[in] count size of buffer (and number of registers) to write
   */
  void write_P(Reg reg, const uint8_t* buf, size_t count)
  {
    spi.begin(this);
    spi.transfer(REG_WRITE | reg);
    spi.write_P(buf, count);
    spi.end();
  }

  /**
   * Register DEVICE_TYPE bitfields (Table 18, pp. 59).
   */
  enum {
    DEVICE_TYPE_RX_TRX = 0x08,
    DEVICE_TYPE_TX = 0x07
  } __attribute__((packed));

  /**
   * Register DEVICE_STATUS bitfields.
   */
  enum {
    FFOVL = 0x80,
    FFUNFL = 0x40,
    RXFFEM = 0x20,
    HEADERR = 0x10,
    FREQERR = 0x08,
    LOCKDET = 0x04,
    CPS = 0x03,
    CPS_IDLE = 0x00,
    CPS_RX = 0x01,
    CPS_TX = 0x10
  } __attribute__((packed));

  /**
   * Register INTERRUPT_STATUS1 bitfields.
   */
  enum {
    IFFERROR = 0x80,
    ITXFFAFULL = 0x40,
    ITXFFAEM = 0x20,
    IRXFFAFULL = 0x10,
    IEXT = 0x08,
    IPKSENT = 0x04,
    IPKVALID = 0x02,
    ICRCERROR = 0x01
  } __attribute__((packed));
 
  /**
   * Register INTERRUPT_STATUS2 bitfields.
   */
  enum {
    ISWDET = 0x80,
    IPREAVAL = 0x40,
    IPREAINVAL = 0x20,
    IRSSI = 0x10,
    IWUT = 0x08,
    ILBD = 0x04,
    ICHIPRDY = 0x02,
    IPOR = 0x01
  } __attribute__((packed));

  /**
   * Register INTERRUPT_ENABLE1 bitfields.
   */
  enum {
    ENFFERR = 0x80,
    ENTXFFAFULL = 0x40,
    ENTXFFAEM = 0x20,
    ENRXFFAFULL = 0x10,
    ENEXT = 0x08,
    ENPKSENT = 0x04,
    ENPKVALID = 0x02,
    ENCRCERROR = 0x01
  } __attribute__((packed));

  /**
   * Register INTERRUPT_ENABLE2 bitfields.
   */
  enum {
    ENSWDET = 0x80,
    ENPREAVAL = 0x40,
    ENPREAINVAL = 0x20,
    ENRSSI = 0x10,
    ENWUT = 0x08,
    ENLBDI = 0x04,
    ENCHIPRDY = 0x02,
    ENPOR = 0x01
  } __attribute__((packed));

  /**
   * Register OPERATING_MODE bitfields.
   */
  enum {
    SWRES = 0x80,
    ENLBD = 0x40,
    ENWT = 0x20,
    X32KSEL = 0x10,
    TXON = 0x08,
    RXON = 0x04,
    PLLON = 0x02,
    XTON = 0x01
  } __attribute__((packed));

  /**
   * Register OPERATING_MODE2 bitfields.
   */
  enum {
    ANTDIV = 0xC0,
    RXMPK = 0x10,
    AUTOTX = 0x08,
    ENLDM = 0x04,
    FFCLRRX = 0x02,
    FFCLRTX = 0x01
  } __attribute__((packed));

  /**
   * Register ADC_CONFIGURATION bitfields.
   */
  enum {
    ADCSTART = 0x80,
    ADCDONE = 0x80,
    ADCSEL = 0x70,
    ADCSEL_INTERNAL_TEMPERATURE_SENSOR = 0x00,
    ADCSEL_GPIO0_SINGLE_ENDED = 0x10,
    ADCSEL_GPIO1_SINGLE_ENDED = 0x20,
    ADCSEL_GPIO2_SINGLE_ENDED = 0x30,
    ADCSEL_GPIO0_GPIO1_DIFFERENTIAL = 0x40,
    ADCSEL_GPIO1_GPIO2_DIFFERENTIAL = 0x50,
    ADCSEL_GPIO0_GPIO2_DIFFERENTIAL = 0x60,
    ADCSEL_GND = 0x70,
    ADCREF = 0x0C,
    ADCREF_BANDGAP_VOLTAGE = 0x00,
    ADCREF_VDD_ON_3 = 0x08,
    ADCREF_VDD_ON_2 = 0x0C,
    ADCGAIN = 0x03
  } __attribute__((packed));

  /**
   * Register ADC_SENSOR_AMP_OFFSET bitfields.
   */
  enum {
    ADCOFFS = 0x0F
  } __attribute__((packed));

  /**
   * Register TEMPERATURE_SENSOR_CALIBRATION bitfields.
   */
  enum {
    TSRANGE = 0xC0,
    TSRANGE_M64_64C = 0x00,
    TSRANGE_M64_192C = 0x40,
    TSRANGE_0_128C = 0x80,
    TSRANGE_M40_216F = 0xC0,
    ENTSOFFS = 0x20,
    ENTSTRIM = 0x10,
    TSTRIM = 0x0F
  } __attribute__((packed));

  /**
   * Register WAKEUP_TIMER_PERIOD1 bitfields.
   */
  enum {
    WTR = 0x3C,
    WTD = 0x03
  } __attribute__((packed));

  /**
   * Register AFC_LOOP_GEARSHIFT_OVERRIDE bitfields.
   */
  enum {
    AFBCD = 0x80,
    ENAFC = 0x40,
    AFCGEARH = 0x38,
    AFCGEARL = 0x07
  } __attribute__((packed));

  /**
   * Register AFC_TIMING_CONTROL bitfields.
   */
  enum {
    SWAIT_TIMER = 0xc0,
    SHWAIT = 0x38,
    ANWAIT = 0x07
  } __attribute__((packed));

 /**
   * Register DATA_ACCESS_CONTROL bitfields.
   */
  enum {
    ENPACRX = 0x80,
    MSBFRST = 0x00,
    LSBFRST = 0x40,
    CRCHDRS = 0x00,
    CRCDONLY = 0x20,
    SKIP2PH = 0x10,
    ENPACTX = 0x08,
    ENCRC = 0x04,
    CRC = 0x03,
    CRC_CCITT = 0x00,
    CRC_CRC_16_IBM = 0x01,
    CRC_IEC_16 = 0x02,
    CRC_BIACHEVA = 0x03
  } __attribute__((packed));

  /**
   * Register HEADER_CONTROL1 bitfields.
   */
  enum {
    BCEN = 0xF0,
    BCEN_NONE = 0x00,
    BCEN_HEADER0 = 0x10,
    BCEN_HEADER1 = 0x20,
    BCEN_HEADER2 = 0x40,
    BCEN_HEADER3 = 0x80,
    HDCH = 0x0F,
    HDCH_NONE = 0x00,
    HDCH_HEADER0 = 0x01,
    HDCH_HEADER1 = 0x02,
    HDCH_HEADER2 = 0x04,
    HDCH_HEADER3 = 0x08,
  } __attribute__((packed));

  /**
   * Register HEADER_CONTROL2 bitfields.
   */
  enum {
    HDLEN = 0x70,
    HDLEN_0 = 0x00,
    HDLEN_1 = 0x10,
    HDLEN_2 = 0x20,
    HDLEN_3 = 0x30,
    HDLEN_4 = 0x40,
    VARPKLEN = 0x00,
    FIXPKLEN = 0x08,
    SYNCLEN = 0x06,
    SYNCLEN_1 = 0x00,
    SYNCLEN_2 = 0x02,
    SYNCLEN_3 = 0x04,
    SYNCLEN_4 = 0x06,
    PREALEN8 = 0x01
  } __attribute__((packed));

  /**
   * Register TX_POWER bitfields.
   */
  enum {
// https://www.sparkfun.com/datasheets/Wireless/General/RFM22B.pdf
    PAPEAKVAL = 0x80,
    PAPEAKEN = 0x40,
    PAPEAKLVL = 0x30,
    PAPEAKLVL6_5 = 0x00,
    PAPEAKLVL7 = 0x10,
    PAPEAKLVL7_5 = 0x20,
    PAPEAKLVL8 = 0x30,
    LNA_SW = 0x08,
    TXPOW = 0x07,
    TXPOW_4X31 = 0x08, // Not used in RFM22B
// For RFM22B:
    TXPOW_1DBM = 0x00,
    TXPOW_2DBM = 0x01,
    TXPOW_5DBM = 0x02,
    TXPOW_8DBM = 0x03,
    TXPOW_11DBM = 0x04,
    TXPOW_14DBM = 0x05, 
    TXPOW_17DBM = 0x06, 
    TXPOW_20DBM = 0x07, 
// RFM23B only:
    RF23B_TXPOW_M8DBM = 0x00, // -8dBm
    RF23B_TXPOW_M5DBM = 0x01, // -5dBm
    RF23B_TXPOW_M2DBM = 0x02, // -2dBm
    RF23B_TXPOW_1DBM = 0x03, // 1dBm
    RF23B_TXPOW_4DBM = 0x04, // 4dBm
    RF23B_TXPOW_7DBM = 0x05, // 7dBm
    RF23B_TXPOW_10DBM = 0x06, // 10dBm
    RF23B_TXPOW_13DBM = 0x07, // 13dBm
// RFM23BP only:
    RF23BP_TXPOW_28DBM = 0x05, // 28dBm
    RF23BP_TXPOW_29DBM = 0x06, // 29dBm
    RF23BP_TXPOW_30DBM = 0x07 // 30dBm
  } __attribute__((packed));

  /**
   * Register MODULATION_CONTROL2 bitfields.
   */
  enum {
    TRCLK = 0xC0,
    TRCLK_NONE = 0x00,
    TRCLK_GPIO = 0x40,
    TRCLK_SDO = 0x80,
    TRCLK_NIRQ = 0xc0,
    DTMOD = 0x30,
    DTMOD_DIRECT_GPIO = 0x00,
    DTMOD_DIRECT_SDI = 0x10,
    DTMOD_FIFO = 0x20,
    DTMOD_PN9 = 0x30,
    ENINV = 0x08,
    FD8 = 0x04,
    MODTYP = 0x30,
    MODTYP_UNMODULATED = 0x00,
    MODTYP_OOK = 0x01,
    MODTYP_FSK = 0x02,
    MODTYP_GFSK = 0x03
  } __attribute__((packed));

  /**
   * Register FREQUENCY_BAND_SELECT bitfields.
   */
  enum {
    SBSEL = 0x40,
    HBSEL = 0x20,
    FB = 0x1F
  } __attribute__((packed));

  /**
   * Register OP_MODE bitfields (Table 24, pp. 63).
   */
  enum Mode {
    STANDBY_MODE = 0x04,
    SLEEP_MODE = 0x00,
    SENSOR_MODE = 0x01,
    FREQUENCY_SYNTHESIZER_MODE = 0x08,
    TRANSMITTER_MODE = 0x0C,
    RECEIVER_MODE = 0x10
  } __attribute__((packed));

  /**
   * Set the given operation mode and wait for mode to become ready.
   * @param[in] mode to set.
   */
  void set(Mode mode);

  /**
   * Handler for interrupt pin. Service interrupt on incoming messages
   * with valid checksum or message transmission completed.
   */
  class IRQPin : public ExternalInterrupt {
  public:
    /**
     * Construct interrupt pin handler for RFM22 on payload receive
     * interrupt.
     * @param[in] pin external interrupt pin.
     * @param[in] mode interrupt mode.
     * @param[in] rf device.
     */
    IRQPin(Board::ExternalInterruptPin pin, InterruptMode mode, RFM22* rf) : 
      ExternalInterrupt(pin, mode),
      m_rf(rf)
    {}
    
    /**
     * @override Interrupt::Handler
     * Signal message has been receive and is available in receive fifo.
     * Or message has been sent and transceiver is ready.
     * @param[in] arg (not used).
     */
    virtual void on_interrupt(uint16_t arg = 0);

    friend class RFM22;
  private:
    RFM22* m_rf;        //!< Device reference.
  };
  
  /** Default configuration. */
  static const uint8_t config[] __PROGMEM;

  IRQPin m_irq;         //!< Interrupt pin and handler.
  volatile bool m_done;     //!< Packet sent flag (may be set by ISR).
  Mode m_opmode;        //!< Current operation mode.
};
#endif
#endif
