/*
 *  INA219.h
 *
 *  Created on: Feb 15, 2016
 *      Author: TimoteT
 *
 *      Functions related to TI's INA219 current monitor IC
 *      Calibration register set up for .01 ohm shunt resistor, 17.4v bus voltage, 4A max current expected
 *
 *      Figure current LSB:
 *      4A for max current expected
 *      32768 = 2^15 for 15 bit
 *      4A / 32768 = .000122 round to .0001 for current LSB 10uA per bit
 *
 *      Figure power LSB:
 *      P_LSB = 20 * I_LSB
 *      P_LSB = 20 * .0001 = .002 or 2mW per bit
 *
 *      Figure calibration register:
 *      CAL = trunc{ .04096 / I_LSB * shunt }
 *      CAL = trunc{ .04096 / .0001 * .01 }
 *      CAL = 40960 = 0xA000
 *
 */

#ifndef INA219_H_
#define INA219_H_

#include "I2C.h"
// Device address  0x40 = A0 and A1 tied to GND
#define INA219_addr  0x80   // write address    1000000 0x40 + read bit set to 0 = 0x80
#define INA219_raddr 0x81   // read address     1000000 0x40 + read bit set to 1 = 0x81

// Registers
#define CONFIG_REG   0x00   // configuration register
#define SVOLT_REG    0x01   // shunt voltage register
#define VOLT_REG     0x02   // bus voltage register
#define POW_REG      0x03   // power register
#define CUR_REG      0x04   // current register
#define CAL_REG      0x05   // calibration register

#define INA219_reset 0x8000  // write this to config register for reset

#define CONFIG       0x241F  // 32V bus range, div 1 for gain, 12bit BADC, 12bit SADC, continuous mode
#define CAL          0xA000  // .0001 ua/bit

unsigned int cal = 0;
unsigned int config = 0;

// Functions

unsigned char INA219_rx_byte(unsigned char addr) {

	unsigned char v;
	i2c_tx_start();
	i2c_tx_byte(INA219_addr);
	i2c_tx_byte(addr);
	i2c_tx_stop();
	i2c_tx_start();
	i2c_tx_byte(INA219_raddr);
	v = i2c_rx_byte();                // Read Byte
	i2c_tx_ack(ACK);
	i2c_tx_stop();
	return v;
}

unsigned int INA219_rx_word(unsigned char addr) {

	unsigned int v1, v2 = 0;
	i2c_tx_start();
	i2c_tx_byte (INA219_addr);
	i2c_tx_byte(addr);
	i2c_tx_stop();
	i2c_tx_start();
	i2c_tx_byte (INA219_raddr);
	v1 = i2c_rx_byte();                // Read MSB
	i2c_tx_ack(ACK);
	v2 = i2c_rx_byte();                // read LSB
	v1 = (v1 << 8) | v2;
	i2c_tx_ack(ACK);
	i2c_tx_stop();
	return v1;
}

void INA219_tx_byte(unsigned char addr, unsigned char data) {

    i2c_tx_start();
    i2c_tx_byte(INA219_addr);
    i2c_tx_byte(addr);
    i2c_tx_byte(data);
    i2c_tx_stop();
}

void INA219_tx_word(unsigned char addr, unsigned int data) {

	unsigned char b1, b2, a = 0;
	b2 = data;
	b1 = data >> 8;
	i2c_tx_start();
	i2c_tx_byte (INA219_addr);
	i2c_tx_byte(addr);
	a = i2c_tx_byte(b1);   // check for ACK

	if (a == 0)
		return;     // got a NACK

	i2c_tx_byte(b2);
	i2c_tx_stop();
}

// write to the calibration and configuration registers to set up INA219 with .01 omh shunt resister
void INA219_init(void) {

	unsigned char b2 = (char)CONFIG;         // 1st byte of configuration register setting
	unsigned char b1 = CONFIG >> 8;          // 2nd byte
	unsigned char a = 0;

	// first read the register to see whats in it should be 0
	config = INA219_rx_word(CONFIG_REG);
	// Then write to the register
	if (config != CONFIG) {

		i2c_tx_start();
		i2c_tx_byte(INA219_addr);
		i2c_tx_byte(CONFIG_REG);
		a = i2c_tx_byte(b1);     // send 1st byte and check for ACK

		if (a == 0)
			return;              // got a NACK

		i2c_tx_byte(b2);
		i2c_tx_stop();
	}

	// Now write to the calibration register
	cal = INA219_rx_word(CAL_REG);
	if (cal != CAL) {

		b2 = (char)CAL;         // 1st byte of calibration register setting
		b1 = CAL >> 8;          // 2nd byte
		i2c_tx_start();
		i2c_tx_byte(INA219_addr);
		i2c_tx_byte(CAL_REG);
		a = i2c_tx_byte(b1);     // send 1st byte and check for ACK

		if (a == 0)
			return;              // got a NACK

		i2c_tx_byte(b2);
		i2c_tx_stop();

	}

	config = INA219_rx_word(CONFIG_REG);   // to double check that values are correct
	cal = INA219_rx_word(CAL_REG);
}

signed int INA219_rx_shunt(void) {

	unsigned int result, sign, comp = 0;
	result = INA219_rx_word(SVOLT_REG);

	// check for sign this should be 12 bit with the 1st 4 MSB's being sign bits
	sign = (result & 0xF000) >> 12;
	if (sign == 1) {                // we are dealing with a negative voltage in 2's compliment form

		comp = ~result;            // not the result to lose the sign bits and get the compliment
		comp = comp + 1;           // add 1 to the compliment for the result
		return comp | 0x8000;      // put the sign back in
	}

	return result;
}

unsigned int INA219_rx_vbus(void) {

	unsigned int V_REG = INA219_rx_word(VOLT_REG);
    return (V_REG >> 3) * 4;                        // 4mv
}

unsigned int INA219_rx_current(void) {

	unsigned int I_REG = INA219_rx_word(CUR_REG);
	return I_REG / 10;                             // I_LSB = .0001
}

unsigned int INA219_rx_power(void) {

	unsigned int P_REG = INA219_rx_word(POW_REG);
	return P_REG * 2;                              // 2mW
}

#endif /* INA219_H_ */
