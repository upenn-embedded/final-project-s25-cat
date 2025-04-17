//original main.c

// #ifndef F_CPU
// #define F_CPU 16000000UL
// #endif

// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include "motor_control.h"
// #include "spi.h"
// #include "ultrasonic.h"
// #include <util/delay.h>
// #include "uart.h"

// #define SPEED 200
// #define DELAY 1000

// int main(void) {
//     uint8_t meas = 0;

//     uart_init();
//     spiInit();
//     motorInit();
//     // Since timer1 is used both for ultrasonic input capture and motor PWM, combined setup will be here
//     DDRB |= (1 << L_R); // set left-reverse motor as output
//     TCCR1A = (1 << WGM10) | (1 << COM1A1); // fast PWM, enables PWM for OC1A, clearing when match
//     TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << ICES1); // fast PWM, prescle by 8, and input capture set on rising edge 
//     TIMSK1 = (1 << ICIE1); // enable input capture

//     sei();

//     while (1) {
//         measureDistance();
//         char insn = SPI_Recv(); // receive data from SPI
//         switch (insn) {
//             case 'F':
//                 forward(SPEED);
//                 break;
//             case 'B':
//                 reverse(SPEED);
//                 break;
//             case 'L':
//                 ccw_rotation(SPEED);
//                 break;
//             case 'R':
//                 cw_rotation(SPEED);
//                 break;
//             case 'S':
//                 brake();
//                 break;
//             case 'D':
//                 // meas = getDistance();
//                 break;
//             default:
//                 brake();

//                 //call pin change interrupt for emergency brake feature with push button 
//         }
//     }

//     return 0;
// }

//main.c with ultrasonic & emergency stop  

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

#define BUTTON_PORT PORTC
#define BUTTON_PIN_REG PINC
#define BUTTON_DDR DDRC
#define BUTTON_BIT PC0
#define BUTTON_PCINT PCINT8  // PC0 maps to PCINT8

volatile uint8_t emergency_stop = 0;

// -------- UART PRINT SETUP --------
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// -------- EMERGENCY STOP BUTTON INIT --------
void pin_change_interrupt_init() {
    BUTTON_DDR &= ~(1 << BUTTON_BIT);     // Set as input
    BUTTON_PORT |= (1 << BUTTON_BIT);     // Enable pull-up

    PCICR |= (1 << PCIE1);                // Enable PCINT[14:8] group
    PCMSK1 |= (1 << BUTTON_PCINT);        // Enable PC0
    sei();                                // Global interrupt enable
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
    stdout = &uart_output;

    spiInit();
    motorInit();
    ultrasonic_init();
    pin_change_interrupt_init();

    // Timer1: used for PWM (OC1A) + Input Capture (ultrasonic)
    DDRB |= (1 << L_R); // Set left-reverse motor as output

    TCCR1A = (1 << WGM10) | (1 << COM1A1); // Fast PWM, OC1A
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << ICES1); // Fast PWM + input capture
    TIMSK1 = (1 << ICIE1); // Enable input capture interrupt

    sei();

    while (1) {
        // Emergency stop override
        if (emergency_stop) {
            brake();
            printf("EMERGENCY STOP triggered!\n");
            emergency_stop = 0;
        }

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
                measureDistance();
                meas = getDistance();
                printf("Distance: %d cm\n", meas);
                _delay_ms(100);
                break;
            default:
                brake();
                break;
        }
    }

    return 0;
}
