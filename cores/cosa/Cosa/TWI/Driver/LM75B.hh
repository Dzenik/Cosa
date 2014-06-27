#ifndef COSA_TWI_DRIVER_LM75B_HH
#define COSA_TWI_DRIVER_LM75B_HH

#include "Cosa/Types.h"
#include "Cosa/TWI.hh"

class LM75B : private TWI::Driver {
public:
    /** Represents the different I2C address possibilities for the LM75B
     */
    enum Address {
        ADDRESS_0 = (0x48 << 1),    /**< A[2:0] pins = 000 */
        ADDRESS_1 = (0x49 << 1),    /**< A[2:0] pins = 001 */
        ADDRESS_2 = (0x4A << 1),    /**< A[2:0] pins = 010 */
        ADDRESS_3 = (0x4B << 1),    /**< A[2:0] pins = 011 */
        ADDRESS_4 = (0x4C << 1),    /**< A[2:0] pins = 100 */
        ADDRESS_5 = (0x4D << 1),    /**< A[2:0] pins = 101 */
        ADDRESS_6 = (0x4E << 1),    /**< A[2:0] pins = 110 */
        ADDRESS_7 = (0x4F << 1)     /**< A[2:0] pins = 111 */
    } __attribute__((packed));

    /** Represents the power mode of the LM75B
     */
    enum PowerMode {
        POWER_NORMAL,   /**< Chip is enabled and samples every 100ms */
        POWER_SHUTDOWN  /**< Chip is in low-power shutdown mode */
  } __attribute__((packed));
    /** Represents OS pin mode of the LM75B
     */
    enum OSMode {
        OS_COMPARATOR,  /**< OS is asserted when the temperature reaches the alert threshold, and de-asserted when the temperature drops below the alert hysteresis threshold */
        OS_INTERRUPT    /**< OS is asserted when the temperature reaches the alert threshold, or drops below the alert hysteresis threshold, and only de-asserted when a register has been read */
    } __attribute__((packed));
 
    /** Represents OS pin polarity of the LM75B
     */
    enum OSPolarity {
        OS_ACTIVE_LOW,  /**< OS is a logic low when asserted, and a logic high when de-asserted */
        OS_ACTIVE_HIGH  /**< OS is a logic high when asserted, and a logic low when de-asserted */
    } __attribute__((packed));
 
    /** Represents OS pin fault queue length of the LM75B
     */
    enum OSFaultQueue {
        OS_FAULT_QUEUE_1,   /**< OS is asserted after 1 fault */
        OS_FAULT_QUEUE_2,   /**< OS is asserted after 2 consecutive faults */
        OS_FAULT_QUEUE_4,   /**< OS is asserted after 4 consecutive faults */
        OS_FAULT_QUEUE_6    /**< OS is asserted after 6 consecutive faults */
    } __attribute__((packed));
    
  LM75B() : TWI::Driver(0x4B)
    {
    }
    /** Probe for the LM75B and indicate if it's present on the bus
     *
     * @returns
     *   'true' if the device exists on the bus,
     *   'false' if the device doesn't exist on the bus.
     */
    bool begin();
 
    /** Get the current power mode of the LM75B
     *
     * @returns The current power mode as a PowerMode enum.
     */
    LM75B::PowerMode powerMode();
 
    /** Set the power mode of the LM75B
     *
     * @param mode The new power mode as a PowerMode enum.
     */
    void powerMode(PowerMode mode);
 
    /** Get the current OS pin mode of the LM75B
     *
     * @returns The current OS pin mode as an OSMode enum.
     */
    LM75B::OSMode osMode();
 
    /** Set the OS pin mode of the LM75B
     *
     * @param mode The new OS pin mode as an OSMode enum.
     */
    void osMode(OSMode mode);
 
    /** Get the current OS pin polarity of the LM75B
     *
     * @returns The current OS pin polarity as an OSPolarity enum.
     */
    LM75B::OSPolarity osPolarity();
 
    /** Set the OS pin polarity of the LM75B
     *
     * @param polarity The new OS pin polarity as an OSPolarity enum.
     */
    void osPolarity(OSPolarity polarity);
 
    /** Get the current OS pin fault queue length of the LM75B
     *
     * @returns The current OS pin fault queue length as an OSFaultQueue enum.
     */
    LM75B::OSFaultQueue osFaultQueue();
 
    /** Set the OS pin fault queue length of the LM75B
     *
     * @param queue The new OS pin fault queue length as an OSFaultQueue enum.
     */
    void osFaultQueue(OSFaultQueue queue);
 
    /** Get the current alert temperature threshold of the LM75B
     *
     * @returns The current alert temperature threshold in °C.
     */
    float alertTemp();
 
    /** Set the alert temperature threshold of the LM75B
     *
     * @param temp The new alert temperature threshold in °C.
     */
    void alertTemp(float temp);
 
    /** Get the current alert temperature hysteresis threshold of the LM75B
     *
     * @returns The current alert temperature hysteresis threshold in °C.
     */
    float alertHyst();
 
    /** Set the alert temperature hysteresis threshold of the LM75B
     *
     * @param temp The new alert temperature hysteresis threshold in °C.
     */
    void alertHyst(float temp);
 
    /** Get the current temperature measurement of the LM75B
     *
     * @returns The current temperature measurement in °C.
     */
    short temp();

protected:
    //I2C register addresses
    enum Register {
        REG_TEMP    = 0x00,
        REG_CONF    = 0x01,
        REG_THYST   = 0x02,
        REG_TOS     = 0x03
    };
 
    //Member variables
    //I2C m_I2C;
    //const int m_ADDR;
 
    //Internal functions
    char read8(char reg);
    void write8(char reg, char data);
    short read16(char reg);
    void write16(char reg, short data);
    float readAlertTempHelper(char reg);
    void writeAlertTempHelper(char reg, float temp);
};

#endif