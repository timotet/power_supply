/*
 * main.c
 *
 *  Created on: Feb 14, 2016
 *      Author: TimoteT
 *
 *  This program uses a MSP430G2231 to monitor voltage, current,
 *  and power on my vintage power supply.
 *  This uses a cheap Wintek LCD for display and a TI INA219 current monitor IC
 *  to do the heavy lifting via I2C.
 *
 *
 */

#include "msp430g2231.h"
#include "I2C.h"
#include "INA219.h"

// LCD stuff ////
#define sendData(data)        send(data, 1)
#define sendInstruction(data) send(data, 0)
#define clearDisplay()        sendInstruction(0x01)
#define goHome()              sendInstruction(0x02)
#define DATAPIN    BIT0                       // p1.0 pin 2 on msp430
#define CLOCKPIN   BIT1                       // p1.1 pin 3
#define ENABLEPIN  BIT2                       // p1.2 pin 4
#define RESETPIN   BIT3                       // p1.3 pin 5

// I2C pull ups
//#define SDA        BIT7                     // !!dont need with hardware pull ups!!
//#define SCL        BIT6

// debug led
#define DEBUG_LED  BIT5                       // p1.5 pin 7
#define DEBUG_ON   P1OUT |= DEBUG_LED
#define DEBUG_OFF  P1OUT &= ~DEBUG_LED

// INA219 stuff
unsigned int power = 0;
unsigned int current = 0;
unsigned int volts = 0;

unsigned char Volts[5] = {0};
unsigned char Current[5] = {0};
unsigned char Power[5] = {0};
unsigned char line[24] = {0};                // for displaying a whole line at once
unsigned char newArray[6] = {0};             // for converted value from mV, mA, mW to V, I, W

//// functions  //////
void delay(unsigned int ms) {    // Delays by the specified Milliseconds
	while (ms--) {
		__delay_cycles(1000);    // set to 1000 for 1 Mhz
	}
}

void send(char data, char registerSelect) {

	char bitCounter = 0;
	while (bitCounter < 8) {
		(data & BIT7) ? (P1OUT |= DATAPIN) : (P1OUT &= ~DATAPIN);
		data <<= 1;
		P1OUT |= CLOCKPIN;
		P1OUT &= ~CLOCKPIN;
		bitCounter++;
	}
	registerSelect ? (P1OUT |= DATAPIN) : (P1OUT &= ~DATAPIN);
	P1OUT &= ~ENABLEPIN;
	delay(3);
	P1OUT |= ENABLEPIN;
	delay(10);
	P1OUT &= ~ENABLEPIN;
}

void sendLine(unsigned char data[], char length) {

	char charIndex = 0;
	while (charIndex < length) {
		sendData(data[charIndex]);
		charIndex++;
	}
}

// This itoa handles negative numbers
int itoa(signed int val, unsigned char *str) {

	unsigned int i = 0;

	if (val < 0) {
		str[0] = '-';
		return 1 + itoa(-val, str + 1);
	}

	if (val / 10) {
		i = itoa(val / 10, str);
	}
	str[i] = val % 10 + '0';
	str[++i] = '\0';

	return i;            // strlen(s), i.e. the next free slot in array
}

// this reworks the array from millivolts to volts or amps or milli anything
// array = pointer to array created by itoa
// nArray = the new array
// count = size of first array
void convertArray(unsigned char *array, unsigned char *nArray,
		unsigned int count) {

	unsigned char index, i = 0;

	for (i = 0; i < count; i++) {  // scan the array for number of actual digits

		if (array[i] != 0x00) {              // the itoa spits out 0x00 for 0
			index += 1;
		}
	}

	switch (index) {

	case 1:                    // .00nnn
		nArray[0] = 0xB7;      // decimal
		nArray[1] = 0x30;      // 0
		nArray[2] = 0x30;
		nArray[3] = array[0];
		nArray[4] = array[1];
		nArray[5] = array[3];
		break;

	case 2:                    // .0nnnn
		nArray[0] = 0xB7;      // decimal
		nArray[1] = 0x30;      // 0
		nArray[2] = array[0];
		nArray[3] = array[1];
		nArray[4] = array[2];
		nArray[5] = array[3];
		break;

	case 3:                    // .nnnnn
		nArray[0] = 0xB7;      // decimal
		nArray[1] = array[0];
		nArray[2] = array[1];
		nArray[3] = array[2];
		nArray[4] = array[3];
		nArray[5] = array[4];
		break;

	case 4:                    // n.nnnn
		nArray[0] = array[0];
		nArray[1] = 0xB7;      // decimal
		nArray[2] = array[1];
		nArray[3] = array[2];
		nArray[4] = array[3];
		nArray[5] = array[4];
		break;

	case 5:                    // nn.nnn
		nArray[0] = array[0];
		nArray[1] = array[1];
		nArray[2] = 0xB7;      // decimal
		nArray[3] = array[2];
		nArray[4] = array[3];
		nArray[5] = array[4];
		break;

	default:
		nArray[0] = 0x00;
		nArray[1] = 0x00;
		nArray[2] = 0x00;
		nArray[3] = 0x00;
		nArray[4] = 0x00;
		nArray[5] = 0x00;

	}
}

// build an array that holds all 24 characters for the lcd
void buildLine(unsigned int volts, unsigned int current, unsigned int power) {

	unsigned int i = 0;
	itoa(volts, Volts);
	itoa(current, Current);
	itoa(power, Power);

	// we know volts is converted into millivolts so lets convert to volts for the LCD
	line[0] = 0x56;                   // 1st element of array = V
	convertArray(Volts, newArray, 5);
	for (i = 1; i < 7; i++) {
		if (newArray[i - 1] == 0x00) { // the itoa spits out 0x00 for 0 but the lcd wants 0x30 for 0
			newArray[i - 1] = 0x30;      // so lets change it
		}
		line[i] = newArray[i - 1];
	}
	line[7] = 0x20;                   // blank
	line[8] = 0x49;                   // I for current
	convertArray(Current, newArray, 5);
	for (i = 9; i < 15; i++) {
		if (newArray[i - 9] == 0x00) {
			newArray[i - 9] = 0x30;
		}
		line[i] = newArray[i - 9];
	}
	line[15] = 0x20;                  // blank
	line[16] = 0x57;                  // W for watts / power
	convertArray(Power, newArray, 5);
	for (i = 17; i < 23; i++) {
		if (newArray[i - 17] == 0x00) {
			newArray[i - 17] = 0x30;
		}
		line[i] = newArray[i - 17];
	}
	line[23] = 0xA4;     // extra line
}

void display_init(void) {
	P1OUT &= ~RESETPIN;                // RESETPIN low
	delay(1);
	P1OUT |= RESETPIN;                 // RESETPIN high
	sendInstruction(0x1C);             // PW   power control
	sendInstruction(0x14);             // DO   display on
	sendInstruction(0x28);             // DC   display control
	sendInstruction(0x4F);             // CN   contrast control
	sendInstruction(0xE0);             // DA   set lower DDRAM address
	clearDisplay();
}

void gpio_init(void) {
	P1DIR |= ENABLEPIN | CLOCKPIN | DATAPIN | RESETPIN | DEBUG_LED | BIT4;   // sets ENABLEPIN,CLOCKPIN,DATAPIN,RESETPIN to outputs and BIT4
	//P1REN |= SCL | SDA;                                                    // Set Pull-Ups on SCL and SDA !!dont need with hardware pull ups!!
	//P1OUT |= SCL | SDA;                                                    // set SCL and SDA high
	P1OUT &= ~(CLOCKPIN | DATAPIN | RESETPIN | DEBUG_LED | BIT4);            // sets CLOCKPIN,RESETPIN,and DATAPIN low and BIT4
	P1OUT |= ENABLEPIN;                                                      // sets ENABLEPIN high
	P2DIR = 0xFF;                                                            // port 2 unused pins as outputs
	P2OUT = 0;                                                               // port 2 set low
}

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;                 // stop watchdog

	DCOCTL = CALDCO_1MHZ;                     // calibrate DCO to 1 mhz
	BCSCTL1 = CALBC1_1MHZ;

	gpio_init();                              // set up gpio
	display_init();                           // initiate display
	i2c_init();                               // set up I2C
	_enable_interrupts();
	INA219_init();                            // initiate INA219

    while(1) {

    DEBUG_ON;
    volts = INA219_rx_vbus();
    current = INA219_rx_current();
    power = INA219_rx_power();
    buildLine(volts, current, power);
    sendLine(line, 24);
    goHome();                         // return cursor to 1st DDRAM posistion

    }
}

///// interrupt /////
#pragma vector = USI_VECTOR              // USI interrupt service routine for I2C
__interrupt void USI_i2c_TXRX (void) {

    USICTL1 &= ~USIIFG;                  // Clear pending flag
    LPM0_EXIT;                           // Return from LPM
}

