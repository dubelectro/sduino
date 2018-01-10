/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef _BV
#define _BV(X) (1<<(X))
#endif


#define NUM_DIGITAL_PINS            25
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p)< 6) ? ((p) + 2) : (-1))


/* on the STM8S the logical pin numbers are really confusing instead
 * of beeing helpful. So maybe it is better to use these Portpin-Names
 * instead?
 */
enum portpin {
	PA1, // 0
	PA2, // 1
	PF4, // 2
	PB5, // 3
	PB4, // 4
	PB3, // 5
	PB2, // 6
	PB1, // 7
	PB0, // 8
	PE5, // 9
	PC1, // 10
	PC2, // 11
	PC3, // 12
	PC4, // 13
	PC5, // 14
	PC6, // 15
	PC7, // 16
	PD0, // 17
	PD1, // 18
	PD2, // 19
	PD3, // 20
	PD4, // 21
	PD5, // 22
	PD6, // 23
	PD7, // 24
};



// PWM on pins 10-13,17,19-22
#define digitalPinHasPWM(p)	( ((p)>=10&(p)<=13) | (p)==17 | ((p)>=19&(p)<=22))


#define PIN_SPI_SS    (PE5)	// 9
#define PIN_SPI_MOSI  (PC6)	// 15
#define PIN_SPI_MISO  (PC7)	// 16
#define PIN_SPI_SCK   (PC5)	// 14

/* SDCC workaround: These const variables wouldn't be replaced by hard
 * constant loads. So use defines instead.
static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
*/
#define SS      PIN_SPI_SS
#define	MOSI    PIN_SPI_MOSI
#define	MISO    PIN_SPI_MISO
#define	SCK     PIN_SPI_SCK

#define PIN_WIRE_SDA    (PB5)	// 3
#define PIN_WIRE_SCL    (PB4)	// 4

/* SDCC workaround
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;
*/
#define	SDA	PIN_WIRE_SDA
#define	SCL	PIN_WIRE_SCL

#define PIN_LED_BUILTIN (PE5)	// sduino: pin for the buildin LED, pin 13
#define PIN_TX	(PD5)		// sduino: pin for TX line, pin 8
#define PIN_RX	(PD6)		// sduino: pin for RX line, pin 7

#define LED_BUILTIN (PE5)	// pin for the buildin LED, pin 13

#define PIN_A0   (PF4)		// 2, Ain12
#define PIN_A1   (PB5)		// 3, Ain5
#define PIN_A2   (PB4)		// 4, Ain4
#define PIN_A3   (PB3)		// 5, Ain3
#define PIN_A4   (PB2)		// 6, Ain2
#define PIN_A5   (PB1)		// 7, Ain1
#define PIN_A6   (PB0)		// 8, Ain0

/* SDCC workaround
static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
*/
#define	A0	PIN_A0
#define	A1	PIN_A1
#define	A2	PIN_A2
#define	A3	PIN_A3
#define	A4	PIN_A4
#define	A5	PIN_A5
#define	A6	PIN_A6

//#define NO_ANALOG	0xff

// Distinguish between ADC channel number and digital pin number.
// Note that for value 6 both ranges overlap and it is used a pin number.
//
// values 0..5: ADC channel number, no conversion
// values 6..15: digital pin numbers, convert to ADC channel number
#define analogPinToChannel(p)	(((p) <= 6) ? ((p) == 0 ? (12) : (6-(p))) : (-1))
extern const uint8_t digitalPinToAnalogChannelMap[];

/*FIXME
#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))
*/

#ifdef ARDUINO_MAIN

// STM8S105K5 breakout board
//
//                      +-/\-+
//                GND 16|    |15 GND
//                 5V 17|    |14 3V3
//      PMW (D12) PC3 18|    |13 PC2  (D11) PMW
//      PMW (D13) PC4 19|    |12 PC1  (D10) PMW
//  SCK     (D14) PC5 20|    |11 PE5  (D9)      NSS
// MOSI     (D15) PC6 21|    |10 PD0  (D8)  PMW
// MISO     (D16) PC7 22|    |9  PB1  (D7)  PMW
//      PMW (D17) PD0 23|    |8  PB2  (D6)  PMW
//          (D18) PD1 24|    |7  PB3  (D5)
//      PMW (D19) PD2 25|    |6  PB4  (D4)
//      PMW (D20) PD3 26|    |5  PB5  (D3)
//      PMW (D21) PD4 27|    |4  PF4  (D2)
// TX       (D22) PD5 28|    |3  PA2  (D1)  SDL
// RX       (D23) PD6 29|    |2  PA1  (D0)  SDA
//      PMW (D24) PD7 30|    |1  NRST
//                      +----+
//
// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	GPIOA_BaseAddress+2,
	GPIOB_BaseAddress+2,
	GPIOC_BaseAddress+2,
	GPIOD_BaseAddress+2,
	GPIOE_BaseAddress+2,
	GPIOF_BaseAddress+2,
/*
	(uint16_t) &GPIOA->DDR,
	(uint16_t) &GPIOB->DDR,
	(uint16_t) &GPIOC->DDR,
	(uint16_t) &GPIOD->DDR,
*/
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	GPIOA_BaseAddress,
	GPIOB_BaseAddress,
	GPIOC_BaseAddress,
	GPIOD_BaseAddress,
	GPIOE_BaseAddress,
	GPIOF_BaseAddress,
/*
	(uint16_t) &GPIOA->ODR,
	(uint16_t) &GPIOB->ODR,
	(uint16_t) &GPIOC->ODR,
	(uint16_t) &GPIOD->ODR,
*/
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	GPIOA_BaseAddress+1,
	GPIOB_BaseAddress+1,
	GPIOC_BaseAddress+1,
	GPIOD_BaseAddress+1,
	GPIOE_BaseAddress+1,
	GPIOF_BaseAddress+1,
/*
	(uint16_t) &GPIOA->IDR,
	(uint16_t) &GPIOB->IDR,
	(uint16_t) &GPIOC->IDR,
	(uint16_t) &GPIOD->IDR,
*/
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PA, //  
	PA, // 2
	PF, // 3
	PB, // 4
	PB, // 5
	PB, // 6
	PB, // 7
	PB, // 8
	PB, // 9
	PE, // 10
	PC, // 11
	PC, // 12
	PC, // 13
	PC, // 14
	PC, // 15
	PC, // 16
	PC, // 17
	PD, // 18
	PD, // 19
	PD, // 20
	PD, // 21
	PD, // 22
	PD, // 23
	PD, // 24
	PD, // 25
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(1), /* 0, port A */
	_BV(2),
	_BV(4), /* 2, port F */
	_BV(5), /* 3, port B */
	_BV(4),
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0), 
	_BV(5), /* 9, port E */ 
	_BV(1), /* 10, port C */
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5), 
	_BV(6),
	_BV(7),
	_BV(0), /* 17, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,	// 0
	NOT_ON_TIMER,	// 1
	NOT_ON_TIMER,	// 2
	NOT_ON_TIMER,	// 3
	NOT_ON_TIMER,	// 4
	NOT_ON_TIMER,	// 5
	NOT_ON_TIMER,	// 6
	NOT_ON_TIMER,	// 7
	NOT_ON_TIMER,	// 8
	NOT_ON_TIMER,	// 9
	TIMER11,	// 10
	TIMER12,	// 11
	TIMER13,	// 12
	TIMER14,	// 13
	NOT_ON_TIMER,	// 14
	NOT_ON_TIMER,	// 15
	NOT_ON_TIMER,	// 16
	TIMER32,	// 17
	NOT_ON_TIMER,	// 18
	TIMER31,	// 19
	TIMER22,	// 20
	TIMER21,	// 21
	NOT_ON_TIMER,	// 22
	NOT_ON_TIMER,	// 23
	NOT_ON_TIMER,	//24
};


const uint8_t digitalPinToAnalogChannelMap[] = {
	12,		// A0	D2	PF4	Ain12
	5,		// A1	D3	PB4	Ain5
	4,		// A2	D4	PB3	Ain4
	3,  	// A3   D5	PB2 Ain3
	2,		// A4	D6	PB1	Ain2
	1,		// A5	D7	PB0	Ain1
	0		// A6	D8	PB0	Ain0
};



#endif

#define NEED_TIMER_11_12
#define NEED_TIMER_31_32

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
