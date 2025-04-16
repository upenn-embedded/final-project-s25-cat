#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef __AVR_ATmega328PB__
#define __AVR_ATmega328PB__
#endif


//#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "spi.h"
#include "motor_control.h"
#include "ultrasonic.h"

// TODO: for some reason the compiler isn't recognizing 
// these pins correctly, and the compiler doesn't like SPI ISRs...
#ifndef SPCR
  #define SPCR _SFR_IO8(0x2D)
#endif

#ifndef SPSR
  #define SPSR _SFR_IO8(0x2E)
#endif

#ifndef SPDR
  #define SPDR _SFR_IO8(0x2F)
#endif

void spiInit() {
    DDRB |= (1 << MISO); // MISO is output
    SPCR = (1 << SPE) | (1 << SPIE); // enable SPI
}

#pragma interrupt_handler SPI_STC_handler:SPI_STC_vect
void SPI_STC_handler(void) {
    uint8_t message = SPDR;
    uint8_t response = 0;
    
    switch (message) {
        case 'F':
            forward(100);
            break;
        case 'R':
            reverse(100);
            break;
        case 'B':
            brake();
            break;
        case 'L':
            ccw_rotation(100);
            break;
        case 'D':
            cw_rotation(100);
            break;
        case 'S':
            response = getDistance();
            break;
    }
    
    SPDR = response;
}
