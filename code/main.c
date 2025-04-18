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
#include <stdio.h>

// -------- CONFIG --------
#define SPEED 200
#define DELAY 1000

#define BUTTON_PIN_REG PINC
#define BUTTON_BIT PC0
#define BUTTON_PCINT PCINT8  // PC0 maps to PCINT8

volatile uint8_t emergency_stop = 0;

// -------- EMERGENCY STOP BUTTON INIT --------
void pin_change_interrupt_init() {
    DDRC &= ~(1 << BUTTON_BIT);     // Set as input
    PORTC |= (1 << BUTTON_BIT);     // Enable pull-up

    PCICR |= (1 << PCIE1);                // Enable PCINT[14:8] group
    PCMSK1 |= (1 << BUTTON_PCINT);        // Enable PC0
}

ISR(PCINT1_vect) {
    _delay_ms(10); // debounce
    if (!(BUTTON_PIN_REG & (1 << BUTTON_BIT))) {
        emergency_stop = 1;
    }
}

// -------- MAIN --------
int main(void) {
    uint8_t meas = 0;

    // Init subsystems
    uart_init();

    spiInit();
    motorInit();
    ultrasonic_timer3_init();
    pin_change_interrupt_init();

    sei();

    while (!emergency_stop) {
        char insn = SPI_Recv(); // receive command
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
                measureDistance();          // Triggers measurement
                meas = getDistance();       // Gets last measured distance
                break;
            default:
                brake();
                break;
        }
    }
    brake();
    printf("EMERGENCY STOP triggered!\n");
    emergency_stop = 1;
    for(;;);

    return 0;
}
