/*
Description:
This is a example code for Sandbox Electronics' I2C/SPI to UART bridge module.
You can get one of those products on
http://sandboxelectronics.com

Version:
V0.1

Release Date:
2014-02-16

Author:
Tiequan Shao          info@sandboxelectronics.com

Lisence:
CC BY-NC-SA 3.0

Please keep the above information when you use this code in your project.
*/

// #define SC16IS750_DEBUG_PRINT
#include <SC16IS750.h>
#include <SPI.h>
#include <Wire.h>

#ifdef USER_WIRE
SC16IS750::SC16IS750(TwoWire &WirePort, uint8_t prtcl, uint8_t addr_sspin)
: m_WirePort(&WirePort)
{
    protocol = prtcl;
    device_address_sspin = addr_sspin;
    peek_flag = 0;
}
#else
SC16IS750::SC16IS750(uint8_t prtcl, uint8_t addr_sspin)
{
    protocol = prtcl;
    device_address_sspin = addr_sspin;
    peek_flag = 0;
}
#endif

SC16IS750::SC16IS750(SPIClass &spi, uint8_t cs_pin, uint32_t spi_freq)
{
    protocol = SC16IS750_PROTOCOL_SPI;
    device_address_sspin = cs_pin;
    m_spi = &spi;
    m_spi_freq = spi_freq;
    peek_flag = 0;
}


int SC16IS750::begin(uint32_t crystal_freq, uint32_t baud)
{
    // Serial.println("1111111111111111");
    m_crystal_freq = crystal_freq;
    if (protocol == SC16IS750_PROTOCOL_I2C)
    {
        // Serial.println("22222222222222");
#ifndef USER_WIRE
        WIRE.begin();
#endif
    }
    else
    {
        ::pinMode(device_address_sspin, OUTPUT);
        ::digitalWrite(device_address_sspin, HIGH);
        // SPI bus is expected to be initialised by the caller.
    };
    if (!isConnected())
    {
        Serial.println("SC16IS750 not detected at address 0x" + String(device_address_sspin, HEX));
        return 1;
    }
    ResetDevice();
    FIFOEnable(1);
    SetBaudrate(baud);
    SetLine(8, 0, 1);
    return 0;
}

int SC16IS750::available(void)
{
    return FIFOAvailableData();
}

int SC16IS750::read(void)
{
    if (peek_flag == 0)
    {
        return ReadByte();
    }
    else
    {
        peek_flag = 0;
        return peek_buf;
    }
}

size_t SC16IS750::write(uint8_t val)
{
    WriteByte(val);
    return 0;
}

void SC16IS750::pinMode(uint8_t pin, uint8_t i_o)
{
    GPIOSetPinMode(pin, i_o);
}

void SC16IS750::digitalWrite(uint8_t pin, uint8_t value)
{
    GPIOSetPinState(pin, value);
}

uint8_t SC16IS750::digitalRead(uint8_t pin)
{
    return GPIOGetPinState(pin);
}

uint8_t SC16IS750::ReadRegister(uint8_t reg_addr)
{
    uint8_t result = 0;
    if (protocol == SC16IS750_PROTOCOL_I2C)
    { // register read operation via I2C

        WIRE.beginTransmission(device_address_sspin);
        WIRE.write((reg_addr << 3));
        WIRE.endTransmission(0);
        WIRE.requestFrom(device_address_sspin, (uint8_t)1);
        result = WIRE.read();
    }
    else if (protocol == SC16IS750_PROTOCOL_SPI)
    { // register read operation via SPI
        SPIClass &spi = m_spi ? *m_spi : SPI;
        spi.beginTransaction(SPISettings(m_spi_freq, MSBFIRST, SPI_MODE0));
        ::digitalWrite(device_address_sspin, LOW);
        spi.transfer(0x80 | (reg_addr << 3));
        result = spi.transfer(0xff);
        ::digitalWrite(device_address_sspin, HIGH);
        spi.endTransaction();
    }

    return result;
}

void SC16IS750::WriteRegister(uint8_t reg_addr, uint8_t val)
{
    if (protocol == SC16IS750_PROTOCOL_I2C)
    { // register read operation via I2C
        WIRE.beginTransmission(device_address_sspin);
        WIRE.write((reg_addr << 3));
        WIRE.write(val);
        WIRE.endTransmission(1);
    }
    else
    {
        SPIClass &spi = m_spi ? *m_spi : SPI;
        spi.beginTransaction(SPISettings(m_spi_freq, MSBFIRST, SPI_MODE0));
        ::digitalWrite(device_address_sspin, LOW);
        spi.transfer(reg_addr << 3);
        spi.transfer(val);
        ::digitalWrite(device_address_sspin, HIGH);
        spi.endTransaction();
    }

    return;
}

int16_t SC16IS750::SetBaudrate(uint32_t baudrate) // return error of baudrate parts per thousand
{
    // FORCE NORMAL REGISTER MAP
    uint8_t lcr = ReadRegister(SC16IS750_REG_LCR);
    lcr &= ~(1 << 7);        // clear DLAB
    WriteRegister(SC16IS750_REG_LCR, lcr);

    uint64_t divisor;
    uint64_t prescaler;
    uint64_t actual_baudrate;
    uint64_t error;
    uint64_t temp_lcr;
    if (baudrate == 0)
    {
        baudrate = 2400;
    }
    uint8_t mcr = ReadRegister(SC16IS750_REG_MCR);
    mcr &= ~(1 << 7);
    WriteRegister(SC16IS750_REG_MCR, mcr);
    if ((ReadRegister(SC16IS750_REG_MCR) & 0x80) == 0)
    { // if prescaler==1
        prescaler = 1;
    }
    else
    {
        prescaler = 4;
    }

    divisor = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (baudrate * 16);

    temp_lcr = ReadRegister(SC16IS750_REG_LCR);
    temp_lcr |= 0x80;
    WriteRegister(SC16IS750_REG_LCR, temp_lcr);
    // write to DLL
    WriteRegister(SC16IS750_REG_DLL, (uint8_t)divisor);
    // write to DLH
    WriteRegister(SC16IS750_REG_DLH, (uint8_t)(divisor >> 8));
    temp_lcr &= 0x7F;
    WriteRegister(SC16IS750_REG_LCR, temp_lcr);

    actual_baudrate = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (16 * divisor);
    error = ((float)actual_baudrate - baudrate) * 1000 / baudrate;
#ifdef SC16IS750_DEBUG_PRINT
    Serial.print("Desired baudrate: ");
    Serial.println(baudrate, DEC);
    Serial.print("Calculated divisor: ");
    Serial.println(divisor, DEC);
    Serial.print("Actual baudrate: ");
    Serial.println(actual_baudrate, DEC);
    Serial.print("Baudrate error: ");
    Serial.println(error, DEC);
#endif

    return error;
}

void SC16IS750::SetLine(uint8_t data_length, uint8_t parity_select, uint8_t stop_length)
{
    uint8_t temp_lcr;
    temp_lcr = ReadRegister(SC16IS750_REG_LCR);
    temp_lcr &= 0xC0; // Clear the lower six bit of LCR (LCR[0] to LCR[5]
#ifdef SC16IS750_DEBUG_PRINT
    Serial.print("LCR Register:0x");
    Serial.println(temp_lcr, DEC);
#endif
    switch (data_length)
    { // data length settings
    case 5:
        break;
    case 6:
        temp_lcr |= 0x01;
        break;
    case 7:
        temp_lcr |= 0x02;
        break;
    case 8:
        temp_lcr |= 0x03;
        break;
    default:
        temp_lcr |= 0x03;
        break;
    }

    if (stop_length == 2)
    {
        temp_lcr |= 0x04;
    }

    switch (parity_select)
    {       // parity selection length settings
    case 0: // no parity
        break;
    case 1: // odd parity
        temp_lcr |= 0x08;
        break;
    case 2: // even parity
        temp_lcr |= 0x18;
        break;
    case 3: // force '1' parity
        temp_lcr |= 0x03;
        break;
    case 4: // force '0' parity
        break;
    default:
        break;
    }

    WriteRegister(SC16IS750_REG_LCR, temp_lcr);
}

void SC16IS750::GPIOSetPinMode(uint8_t pin_number, uint8_t i_o)
{
    uint8_t temp_iodir;

    temp_iodir = ReadRegister(SC16IS750_REG_IODIR);
    if (i_o == OUTPUT)
    {
        temp_iodir |= (0x01 << pin_number);
    }
    else
    {
        temp_iodir &= (uint8_t)~(0x01 << pin_number);
    }

    WriteRegister(SC16IS750_REG_IODIR, temp_iodir);
    return;
}

void SC16IS750::GPIOSetPinState(uint8_t pin_number, uint8_t pin_state)
{
    uint8_t temp_iostate;

    temp_iostate = ReadRegister(SC16IS750_REG_IOSTATE);
    if (pin_state == 1)
    {
        temp_iostate |= (0x01 << pin_number);
    }
    else
    {
        temp_iostate &= (uint8_t)~(0x01 << pin_number);
    }

    WriteRegister(SC16IS750_REG_IOSTATE, temp_iostate);
    return;
}

uint8_t SC16IS750::GPIOGetPinState(uint8_t pin_number)
{
    uint8_t temp_iostate;

    temp_iostate = ReadRegister(SC16IS750_REG_IOSTATE);
    if ((temp_iostate & (0x01 << pin_number)) == 0)
    {
        return 0;
    }
    return 1;
}

uint8_t SC16IS750::GPIOGetPortState(void)
{

    return ReadRegister(SC16IS750_REG_IOSTATE);
}

void SC16IS750::GPIOSetPortMode(uint8_t port_io)
{
    WriteRegister(SC16IS750_REG_IODIR, port_io);
    return;
}

void SC16IS750::GPIOSetPortState(uint8_t port_state)
{
    WriteRegister(SC16IS750_REG_IOSTATE, port_state);
    return;
}

void SC16IS750::SetPinInterrupt(uint8_t io_int_ena)
{
    WriteRegister(SC16IS750_REG_IOINTENA, io_int_ena);
    return;
}

void SC16IS750::ResetDevice(void)
{
    uint8_t reg;

    reg = ReadRegister(SC16IS750_REG_IOCONTROL);
    reg |= 0x08;
    WriteRegister(SC16IS750_REG_IOCONTROL, reg);

    return;
}

void SC16IS750::ModemPin(uint8_t gpio) // gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
    uint8_t temp_iocontrol;

    temp_iocontrol = ReadRegister(SC16IS750_REG_IOCONTROL);
    if (gpio == 0)
    {
        temp_iocontrol |= 0x02;
    }
    else
    {
        temp_iocontrol &= 0xFD;
    }
    WriteRegister(SC16IS750_REG_IOCONTROL, temp_iocontrol);

    return;
}

void SC16IS750::GPIOLatch(uint8_t latch)
{
    uint8_t temp_iocontrol;

    temp_iocontrol = ReadRegister(SC16IS750_REG_IOCONTROL);
    if (latch == 0)
    {
        temp_iocontrol &= 0xFE;
    }
    else
    {
        temp_iocontrol |= 0x01;
    }
    WriteRegister(SC16IS750_REG_IOCONTROL, temp_iocontrol);

    return;
}

bool SC16IS750::isConnected()
{
    if (protocol == SC16IS750_PROTOCOL_SPI)
    {
        return ping() != 0;
    }
    WIRE.beginTransmission(device_address_sspin);
    if (WIRE.endTransmission() != 0)
        return (false); //Sensor did not ACK
    return (true);
}

void SC16IS750::InterruptControl(uint8_t int_ena)
{
    WriteRegister(SC16IS750_REG_IER, int_ena);
}

uint8_t SC16IS750::InterruptPendingTest(void)
{
    return (ReadRegister(SC16IS750_REG_IIR) & 0x01);
}

void SC16IS750::__isr(void)
{
    uint8_t irq_src;

    irq_src = ReadRegister(SC16IS750_REG_IIR);
    irq_src = (irq_src >> 1);
    irq_src &= 0x3F;

    switch (irq_src)
    {
    case 0x06: // Receiver Line Status Error
        break;
    case 0x0c: // Receiver time-out interrupt
        break;
    case 0x04: // RHR interrupt
        break;
    case 0x02: // THR interrupt
        break;
    case 0x00: // modem interrupt;
        break;
    case 0x30: // input pin change of state
        break;
    case 0x10: // XOFF
        break;
    case 0x20: // CTS,RTS
        break;
    default:
        break;
    }
    return;
}

void SC16IS750::FIFOEnable(uint8_t fifo_enable)
{
    uint8_t temp_fcr;

    temp_fcr = ReadRegister(SC16IS750_REG_FCR);

    if (fifo_enable == 0)
    {
        temp_fcr &= 0xFE;
    }
    else
    {
        temp_fcr |= 0x01;
    }
    WriteRegister(SC16IS750_REG_FCR, temp_fcr);

    return;
}

void SC16IS750::FIFOReset(uint8_t rx_fifo)
{
    uint8_t temp_fcr;

    temp_fcr = ReadRegister(SC16IS750_REG_FCR);

    if (rx_fifo == 0)
    {
        temp_fcr |= 0x04;
    }
    else
    {
        temp_fcr |= 0x02;
    }
    WriteRegister(SC16IS750_REG_FCR, temp_fcr);

    return;
}

void SC16IS750::FIFOSetTriggerLevel(uint8_t rx_fifo, uint8_t length)
{
    uint8_t temp_reg;

    temp_reg = ReadRegister(SC16IS750_REG_MCR);
    temp_reg |= 0x04;
    WriteRegister(SC16IS750_REG_MCR, temp_reg); // SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

    temp_reg = ReadRegister(SC16IS750_REG_EFR);
    WriteRegister(SC16IS750_REG_EFR, temp_reg | 0x10); // set ERF[4] to '1' to use the  enhanced features
    if (rx_fifo == 0)
    {
        WriteRegister(SC16IS750_REG_TLR, length << 4); // Tx FIFO trigger level setting
    }
    else
    {
        WriteRegister(SC16IS750_REG_TLR, length); // Rx FIFO Trigger level setting
    }
    WriteRegister(SC16IS750_REG_EFR, temp_reg); // restore EFR register

    return;
}

uint8_t SC16IS750::FIFOAvailableData(void)
{
#ifdef SC16IS750_DEBUG_PRINT
    Serial.print("=====Available data:");
    Serial.println(ReadRegister(SC16IS750_REG_RXLVL), DEC);
#endif
    return ReadRegister(SC16IS750_REG_RXLVL);
    //    return ReadRegister(SC16IS750_REG_LSR) & 0x01;
}

uint8_t SC16IS750::FIFOAvailableSpace(void)
{
    return ReadRegister(SC16IS750_REG_TXLVL);
}

void SC16IS750::WriteByte(uint8_t val)
{
    uint8_t tmp_lsr;
    /*   while ( FIFOAvailableSpace() == 0 ){
   #ifdef  SC16IS750_DEBUG_PRINT
           Serial.println("No available space");
   #endif

       };

   #ifdef  SC16IS750_DEBUG_PRINT
       Serial.println("++++++++++++Data sent");
   #endif
       WriteRegister(SC16IS750_REG_THR,val);
   */
    do
    {
        tmp_lsr = ReadRegister(SC16IS750_REG_LSR);
    } while ((tmp_lsr & 0x20) == 0);

    WriteRegister(SC16IS750_REG_THR, val);
}

int SC16IS750::ReadByte(void)
{
    volatile uint8_t val;
    if (FIFOAvailableData() == 0)
    {
#ifdef SC16IS750_DEBUG_PRINT
        Serial.println("No data available");
#endif
        return -1;
    }
    else
    {

#ifdef SC16IS750_DEBUG_PRINT
        Serial.println("***********Data available***********");
#endif
        val = ReadRegister(SC16IS750_REG_RHR);
        return val;
    }
}

void SC16IS750::EnableTransmit(uint8_t tx_enable)
{
    uint8_t temp_efcr;
    temp_efcr = ReadRegister(SC16IS750_REG_EFCR);
    if (tx_enable == 0)
    {
        temp_efcr |= 0x04;
    }
    else
    {
        temp_efcr &= 0xFB;
    }
    WriteRegister(SC16IS750_REG_EFCR, temp_efcr);

    return;
}

uint8_t SC16IS750::ping()
{
    WriteRegister(SC16IS750_REG_SPR, 0x55);
    if (ReadRegister(SC16IS750_REG_SPR) != 0x55)
    {
        return 0;
    }

    WriteRegister(SC16IS750_REG_SPR, 0xAA);
    if (ReadRegister(SC16IS750_REG_SPR) != 0xAA)
    {
        return 0;
    }

    return 1;
}
/*
void SC16IS750::setTimeout(uint32_t time_out)
{
    timeout = time_out;
}

size_t SC16IS750::readBytes(char *buffer, size_t length)
{
    size_t count=0;
    int16_t tmp;

    while (count < length) {
        tmp = readwithtimeout();
        if (tmp < 0) {
            break;
        }
        *buffer++ = (char)tmp;
        count++;
    }

    return count;
}

int16_t SC16IS750::readwithtimeout()
{
  int16_t tmp;
  uint32_t time_stamp;
  time_stamp = millis();
  do {
    tmp = read();
    if (tmp >= 0) return tmp;
  } while(millis() - time_stamp < timeout);
  return -1;     // -1 indicates timeout
}
*/

void SC16IS750::flush()
{
    uint8_t tmp_lsr;

    do
    {
        tmp_lsr = ReadRegister(SC16IS750_REG_LSR);
    } while ((tmp_lsr & 0x20) == 0);
}

int SC16IS750::peek()
{
    if (peek_flag == 0)
    {
        peek_buf = ReadByte();
        if (peek_buf >= 0)
        {
            peek_flag = 1;
        }
    }

    return peek_buf;
}
