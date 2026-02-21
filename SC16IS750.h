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

#ifndef _SC16IS750_H_
#define _SC16IS750_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

// Device Address

// A:VDD
// B:GND
// C:SCL
// D:SDA
#define SC16IS750_ADDRESS_00 (0X4D) // Address when A0 -> GND, A1 -> GND

// General Registers
#define SC16IS750_REG_RHR (0x00)
#define SC16IS750_REG_THR (0X00)
#define SC16IS750_REG_IER (0X01)
#define SC16IS750_REG_FCR (0X02)
#define SC16IS750_REG_IIR (0X02)
#define SC16IS750_REG_LCR (0X03)
#define SC16IS750_REG_MCR (0X04)
#define SC16IS750_REG_LSR (0X05)
#define SC16IS750_REG_MSR (0X06)
#define SC16IS750_REG_SPR (0X07)
#define SC16IS750_REG_TCR (0X06)
#define SC16IS750_REG_TLR (0X07)
#define SC16IS750_REG_TXLVL (0X08)
#define SC16IS750_REG_RXLVL (0X09)
#define SC16IS750_REG_IODIR (0X0A)
#define SC16IS750_REG_IOSTATE (0X0B)
#define SC16IS750_REG_IOINTENA (0X0C)
#define SC16IS750_REG_IOCONTROL (0X0E)
#define SC16IS750_REG_EFCR (0X0F)

// Special Registers
#define SC16IS750_REG_DLL (0x00)
#define SC16IS750_REG_DLH (0X01)

// Enhanced Registers
#define SC16IS750_REG_EFR (0X02)
#define SC16IS750_REG_XON1 (0X04)
#define SC16IS750_REG_XON2 (0X05)
#define SC16IS750_REG_XOFF1 (0X06)
#define SC16IS750_REG_XOFF2 (0X07)

//
#define SC16IS750_INT_CTS (0X80)
#define SC16IS750_INT_RTS (0X40)
#define SC16IS750_INT_XOFF (0X20)
#define SC16IS750_INT_SLEEP (0X10)
#define SC16IS750_INT_MODEM (0X08)
#define SC16IS750_INT_LINE (0X04)
#define SC16IS750_INT_THR (0X02)
#define SC16IS750_INT_RHR (0X01)

// Application Related

#define SC16IS750_CRYSTCAL_FREQ m_crystal_freq
// #define     SC16IS750_DEBUG_PRINT   (0)
#define SC16IS750_PROTOCOL_I2C (0)
#define SC16IS750_PROTOCOL_SPI (1)

#ifdef __AVR__
#define WIRE Wire
#elif defined(ESP32)
#define USER_WIRE 1
#define WIRE m_WirePort
#else // Arduino Due
#define WIRE Wire1
#endif

class SC16IS750 : public Stream
{
public:
#ifdef USER_WIRE
    SC16IS750(TwoWire &WirePort, uint8_t prtcl = SC16IS750_PROTOCOL_I2C, uint8_t addr_sspin = SC16IS750_ADDRESS_00);
#else
    SC16IS750(uint8_t prtcl = SC16IS750_PROTOCOL_I2C, uint8_t addr_sspin = SC16IS750_ADDRESS_00);
#endif
    int begin(uint32_t crystal_freq, uint32_t baud);
    int read();
    size_t write(uint8_t val);
    int available();
    void pinMode(uint8_t pin, uint8_t io);
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);
    uint8_t ping();
    //	void setTimeout(uint32_t);
    //	size_t readBytes(char *buffer, size_t length);
    int peek();
    void flush();
    uint8_t GPIOGetPortState(void);
    uint8_t InterruptPendingTest(void);
    void SetPinInterrupt(uint8_t io_int_ena);
    void InterruptControl(uint8_t int_ena);
    void ModemPin(uint8_t gpio); // gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
    void GPIOLatch(uint8_t latch);

    bool isConnected();

private:
    uint8_t device_address_sspin;
    uint8_t protocol;
    //	uint32_t timeout;
    int16_t SetBaudrate(uint32_t baudrate);
    uint8_t ReadRegister(uint8_t reg_addr);
    void WriteRegister(uint8_t reg_addr, uint8_t val);
    void SetLine(uint8_t data_length, uint8_t parity_select, uint8_t stop_length);
    void GPIOSetPinMode(uint8_t pin_number, uint8_t i_o);
    void GPIOSetPinState(uint8_t pin_number, uint8_t pin_state);

    uint8_t GPIOGetPinState(uint8_t pin_number);
    void GPIOSetPortMode(uint8_t port_io);
    void GPIOSetPortState(uint8_t port_state);
    void ResetDevice(void);

    void __isr(void);
    void FIFOEnable(uint8_t fifo_enable);
    void FIFOReset(uint8_t rx_fifo);
    void FIFOSetTriggerLevel(uint8_t rx_fifo, uint8_t length);
    uint8_t FIFOAvailableData(void);
    uint8_t FIFOAvailableSpace(void);
    void WriteByte(uint8_t val);
    int ReadByte(void);
    void EnableTransmit(uint8_t tx_enable);
    //	int16_t readwithtimeout();
    int peek_buf;
    uint8_t peek_flag;

#ifdef USER_WIRE
    TwoWire &m_WirePort;
#endif

    uint32_t m_crystal_freq;
};

#endif
