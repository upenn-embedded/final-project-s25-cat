//original ultrasonic.c

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "ultrasonic.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

volatile uint16_t timer1_count = 0;
volatile uint8_t measurementReady = 0;
uint8_t distanceCm = 0;

// ----- TIMER 3 SETUP FOR ULTRASONIC -----
void ultrasonic_timer3_init() {
    TCCR3A = 0;
    TCCR3B = (1 << CS31);        // Prescaler = 8 (tick = 0.5 µs)
    TIMSK3 = (1 << ICIE3);       // Enable input capture interrupt
    TCCR3B |= (1 << ICES3);      // Trigger on rising edge first
}

// ----- TRIGGER PULSE -----
void send_trigger_pulse() {
    DDRD |= (1 << TRIG_PIN);
    PORTD &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
}


// ----- INPUT CAPTURE ISR -----
ISR(TIMER3_CAPT_vect) {
    static uint16_t start_time = 0;

    if (TCCR3B & (1 << ICES3)) {
        start_time = ICR3;
        TCCR3B &= ~(1 << ICES3);
    } else {
        timer_count = ICR3 - start_time;
        measurementReady = 1;
        TCCR3B |= (1 << ICES3);
    }
}


void measureDistance() {
    send_trigger_pulse();
    _delay_ms(60);
    if (measurementReady) {
        uint16_t duration = timer_count / 2;  // in µs
        uint16_t distance_cm = duration / 58;
        last_distance = distance_cm;
        measurementReady = 0;
    }
}

uint8_t getDistance() {
    return distanceCm;
}
