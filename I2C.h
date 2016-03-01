/*
 * I2C.h
 *
 *This library is for I2C it works with g series chips that have the USI feature
 *This code is mostly just copied from fj604's i2c post on the 43oh forum
 *
 *  Created on: Feb 25, 2012
 *      Author: TimoteT
 */
#include "msp430g2231.h"
#ifndef I2C_H_
#define I2C_H_

#define ACK     0x00
#define NACK    0xFF

void i2c_init(void)
{
    USICTL0 = USIPE7 | USIPE6 | USIMST | USISWRST;    // SDA enable, SCL enable, Master mode, held in reset state
    USICTL1 = USII2C  | USIIE;                       // I2C mode, Interrupt Enable;
    USICKCTL = USIDIV_3 | USISSEL_2 | USICKPL;        // Clock / 8 = ~125khz , SMCLK, SCL inactive is high
    USICTL0 &= ~USISWRST;                             // Release from reset
    USICTL1 &= ~USIIFG;                               // Clear pending flag
}

void  i2c_tx_start(void)
{
    P1OUT |= BIT0;             // start sequence
    USISRL = 0x00;             // Load USISRL Lower Byte Shift Register MSB with 0 for i2c START
    USICTL0 |= USIGE + USIOE;  // Force Output Latch, And Enable Data Output Bit (High to Low SDA while SCL is High)
    USICTL0 &= ~USIGE;         // Clear Output Latch (Return to Clock Control)
}

void i2c_tx_rpt_start(void)
{
	P1OUT |= BIT0 ;             // start sequence
    USICTL0 |= USIOE;           // SDA enable
    USISRL = 0xFF;              // Set MSB of USISRL to 1
    USICNT = 1;                 // Transmit 1 bit for a surplus clock cycle
    LPM0;                       //LPM0 until interrupt
    i2c_tx_start();
}

void i2c_tx_stop(void)
{
	USICTL0 |= USIOE;              // SDA enable
    USISRL = 0;                    // load register with 0
    USICNT = 1;                    // Transmit 1 bit
    LPM0;                          // LPM0 until interrupt
    USISRL = 0xFF;                 // load register with 11111111
    USICTL0 |= USIGE;              // Latch enable
    USICTL0 &= ~(USIGE + USIOE);   // Latch disable, SDA disable
    P1OUT &= ~BIT0;                // end of sequence
}

unsigned char i2c_tx_byte(unsigned char byte)
{
    USICTL0 |= USIOE;          // Enable Data Output (Turn SDA into Output)
    USISRL = byte;             // Load USISRL Lower Byte Shift Register with 8 Bit data (Byte)
    USICNT = 8;                // Load USICNT Counter with number of Bits to Send. USIIFG Auto-Cleared

    LPM0;                      // LPM0 until interrupt

    USICTL0 &= ~USIOE;          // SDA disable
    USICNT = 1;                 // Receive ACK bit

    LPM0;                      // LPM0 until interrupt
    byte = USISRL;             // LSB of USISRL Holds Ack Status of 0 = ACK (0x00) or 1 = NACK (0x01)

    return byte;
}

unsigned char i2c_rx_byte(void)
{
    unsigned char v;
    USICTL0 &= ~USIOE;     // enable data input (Turn SDA into input)
    USICNT = 8;            // bits to read
    LPM0;                  // LPM0 until interrupt
    v = USISRL;
    return v;
}

void i2c_tx_ack(unsigned char ack)
{
    USICTL0 |= USIOE;         // enable data output
    USISRL = ack;             // Send ACK/NACK bit
    USICNT = 1;               // load 1 bit
    LPM0;                     // LPM0 until interrupt
}


#endif /* I2C_H_ */
