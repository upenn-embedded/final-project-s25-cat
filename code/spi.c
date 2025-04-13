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
    DDRB |= (1 << MISO); // MISO is output
    SPCR = (1 << SPE) | (1 << SPIE); // Enable SPI
}

ISR(SPI_STC_vect) {
    // TODO: disable interrupts
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
            ccw_rotation(100);
            break;
        case 'D':
        // go right
            cw_rotation(100);
            break;
        case 'S':
        // send sensor input
            response = getDistance();
            break;
    }

    // send response back
    SPDR = response;
}