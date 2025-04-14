#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define F_CPU 16000000UL

#define TRIG_PIN PD4
#define ECHO_PIN PD6


volatile uint16_t timer1_count = 0;
volatile uint8_t measurementReady = 0;
uint8_t distanceCm = 0;

// ----- TRIGGER PULSE -----
void send_trigger_pulse() {
    DDRD |= (1 << TRIG_PIN);
    PORTD &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
}


ISR(TIMER1_CAPT_vect) {
    static uint16_t start_time = 0;
    
    if (TCCR1B & (1 << ICES1)) {
        // Rising edge detected: save the time
        start_time = ICR1;
        TCCR1B &= ~(1 << ICES1); // Switch to capture falling edge
    } else {
        // Falling edge detected: calculate pulse width
        timer1_count = ICR1 - start_time;
        measurementReady = 1;
        TCCR1B |= (1 << ICES1); // Switch back to capture rising edge for the next measurement
    }
}


void measureDistance() {
    DDRD &= ~(1 << ECHO_PIN);   // set PD6 as input
    PORTD &= ~(1 << ECHO_PIN);  // disable pull-up

    measurementReady = 0;
    send_trigger_pulse();
    _delay_ms(60);
    uint16_t duration_us = timer1_count / 2;
    distanceCm = duration_us / 58;
    
}

uint8_t getDistance() {
    return distanceCm;
}