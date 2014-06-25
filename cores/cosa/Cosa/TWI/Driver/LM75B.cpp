#include "Cosa/TWI/Driver/LM75B.hh"
#include "Cosa/Trace.hh"

bool
LM75B::begin()
{
    uint8_t data = 0;
    if (!twi.begin(this)) return (false);
    int count = twi.read(&data, 1);
    twi.end();
    if (count != sizeof(data)) { return (false); }
    else { return (true); }
}                      
 
LM75B::PowerMode 
LM75B::powerMode()
{
    //Read the 8-bit register value
    char value = read8(REG_CONF);
 
    //Return the status of the SHUTDOWN bit
    if (value & (1 << 0))
        return POWER_SHUTDOWN;
    else
        return POWER_NORMAL;
}
 
void
LM75B::powerMode(PowerMode mode)
{
    //Read the current 8-bit register value
    char value = read8(REG_CONF);
 
    //Set or clear the SHUTDOWN bit
    if (mode == POWER_SHUTDOWN)
        value |= (1 << 0);
    else
        value &= ~(1 << 0);
 
    //Write the value back out
    write8(REG_CONF, value);
}
 
LM75B::OSMode
LM75B::osMode()
{
    //Read the 8-bit register value
    char value = read8(REG_CONF);
 
    //Return the status of the OS_COMP_INT bit
    if (value & (1 << 1))
        return OS_INTERRUPT;
    else
        return OS_COMPARATOR;
}
 
void
LM75B::osMode(OSMode mode)
{
    //Read the current 8-bit register value
    char value = read8(REG_CONF);
 
    //Set or clear the OS_COMP_INT bit
    if (mode == OS_INTERRUPT)
        value |= (1 << 1);
    else
        value &= ~(1 << 1);
 
    //Write the value back out
    write8(REG_CONF, value);
}
 
LM75B::OSPolarity
LM75B::osPolarity()
{
    //Read the 8-bit register value
    char value = read8(REG_CONF);
 
    //Return the status of the OS_POL bit
    if (value & (1 << 2))
        return OS_ACTIVE_HIGH;
    else
        return OS_ACTIVE_LOW;
}
 
void
LM75B::osPolarity(OSPolarity polarity)
{
    //Read the current 8-bit register value
    char value = read8(REG_CONF);
 
    //Set or clear the OS_POL bit
    if (polarity == OS_ACTIVE_HIGH)
        value |= (1 << 2);
    else
        value &= ~(1 << 2);
 
    //Write the value back out
    write8(REG_CONF, value);
}
 
LM75B::OSFaultQueue
LM75B::osFaultQueue()
{
    //Read the 8-bit register value
    char value = read8(REG_CONF);
 
    //Return the status of the OS_F_QUE bits
    if ((value & (1 << 3)) && (value & (1 << 4)))
        return OS_FAULT_QUEUE_6;
    else if (!(value & (1 << 3)) && (value & (1 << 4)))
        return OS_FAULT_QUEUE_4;
    else if ((value & (1 << 3)) && !(value & (1 << 4)))
        return OS_FAULT_QUEUE_2;
    else
        return OS_FAULT_QUEUE_1;
}
 
void
LM75B::osFaultQueue(OSFaultQueue queue)
{
    //Read the current 8-bit register value
    char value = read8(REG_CONF);
 
    //Clear the old OS_F_QUE bits
    value &= ~(3 << 3);
 
    //Set the new OS_F_QUE bits
    if (queue == OS_FAULT_QUEUE_2)
        value |= (1 << 3);
    else if (queue == OS_FAULT_QUEUE_4)
        value |= (2 << 3);
    else if (queue == OS_FAULT_QUEUE_6)
        value |= (3 << 3);
 
    //Write the value back out
    write8(REG_CONF, value);
}
 
float
LM75B::alertTemp()
{
    //Use the 9-bit helper to read the TOS register
    return readAlertTempHelper(REG_TOS);
}
 
void
LM75B::alertTemp(float temp)
{
    //Use the 9-bit helper to write to the TOS register
    return writeAlertTempHelper(REG_TOS, temp);
}
 
float
LM75B::alertHyst()
{
    //Use the 9-bit helper to read the THYST register
    return readAlertTempHelper(REG_THYST);
}
 
void
LM75B::alertHyst(float temp)
{
    //Use the 9-bit helper to write to the THYST register
    return writeAlertTempHelper(REG_THYST, temp);
}

short
LM75B::temp()
{
    //Signed return value
    short value;
 
    //Read the 11-bit raw temperature value
    value = read16(REG_TEMP);
    trace << PSTR("val: ") << bin << value << endl;
    value >>= 5;
    trace << PSTR("val>>5: ") << bin << value << endl;
 
    //Sign extend negative numbers
    if (value & (1 << 10))
        value |= 0xFC00;
 
    //Return the temperature in °C
    return value; //* 0.125;
}
/* 
LM75B::operator
float()
{
    //Return the current temperature reading
    return temp();
}
*/ 
char
LM75B::read8(char reg)
{
    if (!twi.begin(this)) return (false);
    //Select the register
    twi.write(&reg, 1);
 
    //Read the 8-bit register
    twi.read(&reg, 1);
    twi.end();
    //Return the byte
    return reg;
}
 
void
LM75B::write8(char reg, char data)
{
    //Create a temporary buffer
    char buff[2];
 
    //Load the register address and 8-bit data
    buff[0] = reg;
    buff[1] = data;
 
    if (!twi.begin(this)) return;
    //Write the data
    twi.write(&buff, sizeof(buff));
    twi.end();
}
 
short
LM75B::read16(char reg)
{
    //Create a temporary buffer
    char buff[2];
 
    if (!twi.begin(this)) return (false);
    //Select the register
    twi.write(&reg, sizeof(reg));
 
    //Read the 16-bit register
    twi.read(&buff, sizeof(buff));
    twi.end();
    //Return the combined 16-bit value
    return (buff[0] << 8) | buff[1];
}
 
void
LM75B::write16(char reg, short data)
{
    //Create a temporary buffer
    char buff[3];
 
    //Load the register address and 16-bit data
    buff[0] = reg;
    buff[1] = data >> 8;
    buff[2] = data;
 
    if (!twi.begin(this)) return;
    //Write the data
    twi.write(&buff, sizeof(buff));
    twi.end();
}
 
float
LM75B::readAlertTempHelper(char reg)
{
    //Signed return value
    short value;
 
    //Read the 9-bit raw temperature value
    value = read16(reg) >> 7;
 
    //Sign extend negative numbers
    if (value & (1 << 8))
        value |= 0xFF00;
 
    //Return the temperature in °C
    return value * 0.5;
}
 
void
LM75B::writeAlertTempHelper(char reg, float temp)
{
    //Range limit temp
    if (temp < -55.0)
        temp = -55.0;
    else if (temp > 125.0)
        temp = 125.0;
 
    //Extract and shift the signed integer
    short value = temp * 2;
    value <<= 7;
 
    //Send the new value
    write16(reg, value);
}