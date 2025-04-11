#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "spi.h"
#include "motor_control_modified_pins.h"
#include "ultrasonic.h"

void spiInit() {
    // set MISO as output and init SPI
    DDRB |= (1 << MISO);
    SPCR = (1 << SPE) | (1 << SPIE);
}

ISR(SPI_STC_vect) {
    uint8_t message = SPDR;
    uint8_t response = 0;

    switch (message) {
        case 'F':
        // go forward
            forward(100);
            break;
        case 'R':
        // go back
            reverse(100);
            break;
        case 'B':
        // break
            brake();
            break;
        case 'L':
        // go left
            reverse_left(100);
            forward_right(100);
            break;
        case 'D':
        // go right
            forward_left(100);
            reverse_right(100);
            break;
        case 'S':
        // send sensor input
        // TODO: the type casting is very weird, need to fix and figure out actual dims
            uint16_t cmDistance = measureDistance();
            response = cmDistance; // TODO: oopsie response is actually 8bits
            // @Chekayli what was the max distance we got out of your ultrasonic sensor
            break;
    }

    // send response back
    SPDR = response;
}