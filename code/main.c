#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor_control.h"
#include "spi.h"
#include "ultrasonic.h"
#include <util/delay.h>
#include "uart.h"

#define SPEED 200
#define DELAY 1000

int main(void) {
    uint8_t meas = 0;

    uart_init();
    spiInit();
    motorInit();
    // Since timer1 is used both for ultrasonic input capture and motor PWM, combined setup will be here
    DDRB |= (1 << L_R); // set left-reverse motor as output
    TCCR1A = (1 << WGM10) | (1 << COM1A1); // fast PWM, enables PWM for OC1A, clearing when match
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << ICES1); // fast PWM, prescle by 8, and input capture set on rising edge 
    TIMSK1 = (1 << ICIE1); // enable input capture

    sei();

    while (1) {
        measureDistance();
        char insn = SPI_Recv(); // receive data from SPI
        switch (insn) {
            case 'F':
                forward(SPEED);
                break;
            case 'B':
                reverse(SPEED);
                break;
            case 'L':
                ccw_rotation(SPEED);
                break;
            case 'R':
                cw_rotation(SPEED);
                break;
            case 'S':
                brake();
                break;
            case 'D':
                // meas = getDistance();
                break;
            default:
                brake();
        }
    }

    return 0;
}