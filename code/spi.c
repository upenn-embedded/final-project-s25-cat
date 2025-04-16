#ifndef F_CPU
#define F_CPU 16000000UL
#endif


//#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "spi.h"

void spiInit() {
//    SPCR0 = (1<<SPE) | (1 << SPIE); // enable SPI, enable SPI interrupts
    DDRB &= ~((1 << SCK) | (1 << MOSI) | (1 << CS));   // set these to inputs since atmega is peripheral
    DDRB |= (1 << MISO); // MISO is output
}

//int main() {
//    spiInit();
//    DDRD |= (1 << PD7);
//    
////    sei();
//    while (1) {
//        SPI_Recv();
//        PORTD ^= (1 << PD7);
////        _delay_ms(500);
//    }
//    return 0;
//}

char SPI_Recv(void) {
    while(!(SPSR0 & (1 << SPIF)));
    char resp = SPDR0;
    SPDR0 = 5;
    return resp;
}
ISR(SPI_STC_vect) {
    PORTD ^= (1 << PD7);
    uint8_t message = SPDR0;
    uint8_t response = 0;
    
    switch (message) {
        case 'F':
            response = 1;
            break;
        case 'R':
            response = 2;
            break;
        case 'B':
            response = 3;
            break;
        case 'L':
            response = 4;
            break;
        case 'D':
            response = 5;
            break;
        case 'S':
            response = 6;
            break;
    }
    
    SPDR0 = response;
}

