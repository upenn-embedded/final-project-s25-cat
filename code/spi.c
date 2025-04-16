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
#include "uart.h"
#include "ultrasonic.h"

void spiInit() {
    SPCR0 = (1 << SPE); // enable SPI, not interrupts since polling
    DDRB &= ~((1 << SCK) | (1 << MOSI) | (1 << CS));   // set these to inputs since atmega is peripheral
    DDRB |= (1 << MISO); // MISO is output
}

int main() {
    uart_init();
    spiInit();
    
    while (1) {
        char received = SPI_Recv();
    }
    return 0;
}

char SPI_Recv(void) {
    while(!(SPSR0 & (1 << SPIF)));
    char message = SPDR0;
    uint8_t response = getDistance(); // preload distance
    
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
            // response = 6;
            break;
    }
    
    SPDR0 = response;
    return message;
}
