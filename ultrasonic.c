#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// ----- UART (for debugging, optional) -----
#include "uart.h"

// ----- CONSTANTS -----
#define F_CPU 16000000UL
#define BAUD 9600
#define BAUD_PRESCALE (((F_CPU / (BAUD * 16UL))) - 1)

#define TRIG_PIN PD4
#define ECHO_PIN PB0

volatile uint16_t timer_count = 0;
volatile uint8_t measurementReady = 0;
volatile uint16_t last_distance = 0;

// ----- TIMER SETUP FOR ULTRASONIC -----
void ultrasonic_timer1_init() {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);        // Prescaler = 8 (tick = 0.5 µs)
    TIMSK1 = (1 << ICIE1);       // Enable input capture interrupt
    TCCR1B |= (1 << ICES1);      // Trigger on rising edge first
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
ISR(TIMER1_CAPT_vect) {
    static uint16_t start_time = 0;

    if (TCCR1B & (1 << ICES1)) {
        start_time = ICR1;
        TCCR1B &= ~(1 << ICES1);
    } else {
        timer_count = ICR1 - start_time;
        measurementReady = 1;
        TCCR1B |= (1 << ICES1);
    }
}

// ----- PWM MOTOR SETUP -----
void timer0_init() {
    DDRD |= (1 << PD5) | (1 << PD6);
    TCCR0A = (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS01);
}

void timer1_pwm_init() {
    DDRB |= (1 << PB1) | (1 << PB2);
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11);
}

void enable_pwm_IN1() {
    TCCR0A |= (1 << COM0A1);
    TCCR1A |= (1 << COM1A1);
}

void disable_pwm_IN1() {
    TCCR0A &= ~(1 << COM0A1);
    TCCR1A &= ~(1 << COM1A1);
}

void enable_pwm_IN2() {
    TCCR1A |= (1 << COM1B1);
    TCCR0A |= (1 << COM0B1);
}

void disable_pwm_IN2() {
    TCCR1A &= ~(1 << COM1B1);
    TCCR0A &= ~(1 << COM0B1);
}

// ----- MOTOR CONTROL -----
void forward_right(uint8_t speed) {
    TCCR1A &= ~(1 << COM1B1);
    DDRB |= (1 << PB2);
    PORTB &= ~(1 << PB2);

    TCCR0A |= (1 << COM0A1);
    OCR0A = speed;
}

void reverse_right(uint8_t speed) {
    TCCR0A &= ~(1 << COM0A1);
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);

    TCCR1A |= (1 << COM1B1);
    OCR1B = speed;
}

void brake_right() {
    disable_pwm_IN1();
    disable_pwm_IN2();
    DDRD |= (1 << PD6);
    DDRB |= (1 << PB2);
    PORTD |= (1 << PD6);
    PORTB |= (1 << PB2);
}

void forward_left(uint8_t speed) {
    TCCR1A &= ~(1 << COM1B1);
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);

    TCCR0A |= (1 << COM0B1);
    OCR0B = speed;
}

void reverse_left(uint8_t speed) {
    TCCR0A &= ~(1 << COM0B1);
    DDRD |= (1 << PD5);
    PORTD &= ~(1 << PD5);

    TCCR1A |= (1 << COM1A1);
    OCR1A = speed;
}

void brake_left() {
    disable_pwm_IN1();
    disable_pwm_IN2();
    DDRD |= (1 << PD5);
    DDRB |= (1 << PB1);
    PORTD |= (1 << PD5);
    PORTB |= (1 << PB1);
}

void forward(uint8_t speed) {
    forward_left(speed);
    forward_right(speed);
}

void reverse(uint8_t speed) {
    reverse_left(speed);
    reverse_right(speed);
}

void brake() {
    brake_left();
    brake_right();
}

// ----- CW ROTATION PLACEHOLDER -----
void cw_rotation(uint8_t speed, uint16_t angle) {
    forward_left(speed);
    _delay_ms(20000);
}

// ----- MAIN PROGRAM -----
int main(void) {
    uart_init();
    timer0_init();
    timer1_pwm_init();
    ultrasonic_timer1_init();

    DDRB &= ~(1 << ECHO_PIN);   // ECHO as input
    sei();  // Enable global interrupts

    while (1) {
        send_trigger_pulse();
        _delay_ms(60);

        if (measurementReady) {
            uint16_t duration = timer_count / 2;  // in µs
            uint16_t distance_cm = duration / 58;
            last_distance = distance_cm;
            measurementReady = 0;

            printf("Distance: %d cm\n", distance_cm);

            if (distance_cm < 10) {
                brake();
            } else {
                forward(100);
            }
        }
    }
}
