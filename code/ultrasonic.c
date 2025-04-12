#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define TRIG_PIN PD4
#define ECHO_PIN PB0

volatile uint16_t timer3_count = 0;
volatile uint8_t measurementReady = 0;
uint8_t distanceCm = 0;

void ultrasonicInit(void) {
    TCCR3A = 0;
    TCCR3B = (1 << CS31) | (1 << ICES3); // prescaler 8, rising edge
    TIMSK3 = (1 << ICIE3); // Enable input capture interrupt for Timer3
    
    // (If needed, set PORTMUX for Timer3 to map ICP3 to PB0
    // PORTMUX.TIMER3 = PORTMUX_T3_DEFAULT_gc;
    // TODO: Consult datasheet ;-;
}


static void send_trigger_pulse(void) {
    DDRD |= (1 << TRIG_PIN); // set TRIG to output
    // Ensure trigger is low.
    PORTD &= ~(1 << TRIG_PIN); // default low
    _delay_us(2);
    PORTD |= (1 << TRIG_PIN); // Set trigger high for 10 µs
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
}


ISR(TIMER3_CAPT_vect) {
    static uint16_t start_time = 0;
    
    if (TCCR3B & (1 << ICES3)) {
        // Rising edge detected: save the time
        start_time = ICR3;
        TCCR3B &= ~(1 << ICES3); // Switch to capture falling edge
    } else {
        // Falling edge detected: calculate pulse width
        timer3_count = ICR3 - start_time;
        measurementReady = 1;
        TCCR3B |= (1 << ICES3); // Switch back to capture rising edge for the next measurement
    }
}


uint16_t measureDistance(void) {
    send_trigger_pulse();
    _delay_ms(60);
    uint16_t duration_us = timer3_count / 2;
    uint16_t distance_cm = duration_us / 58;
    measurementReady = 0;
    return distance_cm;
}