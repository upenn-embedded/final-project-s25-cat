//original ultrasonic.c

// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <util/delay.h>
// #include <stdint.h>
// #include "ultrasonic.h"

// #ifndef F_CPU
// #define F_CPU 16000000UL
// #endif

// volatile uint16_t timer1_count = 0;
// volatile uint8_t measurementReady = 0;
// uint8_t distanceCm = 0;

// // ----- TRIGGER PULSE -----
// void send_trigger_pulse() {
//     DDRD |= (1 << TRIG_PIN);
//     PORTD &= ~(1 << TRIG_PIN);
//     _delay_us(2);
//     PORTD |= (1 << TRIG_PIN);
//     _delay_us(10);
//     PORTD &= ~(1 << TRIG_PIN);
// }


// ISR(TIMER1_CAPT_vect) {
//     static uint16_t start_time = 0;
    
//     if (TCCR1B & (1 << ICES1)) {
//         // Rising edge detected: save the time
//         start_time = ICR1;
//         TCCR1B &= ~(1 << ICES1); // Switch to capture falling edge
//     } else {
//         // Falling edge detected: calculate pulse width
//         timer1_count = ICR1 - start_time;
//         measurementReady = 1;
//     TCCR1B |= (1 << ICES1); // Switch back to capture rising edge for the next measurement
//     }
// }


// void measureDistance() {
//     send_trigger_pulse();
//     _delay_ms(60);
//     uint16_t duration_us = timer1_count / 2;
//     distanceCm = duration_us / 58;
//     measurementReady = 0;
// }

// uint8_t getDistance() {
//     return distanceCm;
// }


//POLLING BASED ULTRASONIC.C CODE 
#include <avr/io.h>
#include <util/delay.h>
#include "ultrasonic.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

static uint8_t distanceCm = 0;

void ultrasonic_init() {
    // TRIG as output
    DDRD |= (1 << TRIG_BIT);
    PORTD &= ~(1 << TRIG_BIT);

    // ECHO as input
    DDRB &= ~(1 << ECHO_BIT);

    // Timer1 setup (normal mode, prescaler 8 = 0.5 us ticks)
    // TCCR1A = 0;
    // TCCR1B = (1 << CS11); // prescaler = 8
}

void send_trigger_pulse() {
    PORTD &= ~(1 << TRIG_BIT);
    _delay_us(2);
    PORTD |= (1 << TRIG_BIT);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_BIT);
}

void measureDistance() {
    uint16_t start_time = 0;
    uint16_t end_time = 0;

    send_trigger_pulse();

    // Wait for echo to go HIGH (start of pulse)
    while (!(PINB & (1 << ECHO_BIT)));

    // Start timing
    TCNT1 = 0;

    // Wait for echo to go LOW (end of pulse)
    while (PINB & (1 << ECHO_BIT));

    end_time = TCNT1;

    // Convert to distance (tick = 0.5 us)
    uint16_t duration_us = end_time / 2;
    distanceCm = duration_us / 58;
}

uint8_t getDistance() {
    return distanceCm;
}
