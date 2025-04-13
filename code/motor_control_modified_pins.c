#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "motor_control_modified_pins.h"

void motorInit() {
    timer0_init();
    timer2_init();
}

// ----- TIMER INITIALIZATIONS -----
void timer0_init(void) {
    DDRD |= (1 << L_F) | (1 << R_F);
    TCCR0A = (1 << WGM00) | (1 << WGM01); // fast PWM
    TCCR0B = (1 << CS01); // prescale by 8
}

void timer2_init(void) {
    DDRD |= (1 << R_R);
    TCCR2A = (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS21);
    OCR2B = 0;
}

// ----- PWM CONTROL HELPERS -----
void enable_pwm_IN1(void) {
    TCCR0A |= (1 << COM0A1);
    TCCR0A |= (1 << COM0B1);
}

void disable_pwm_IN1(void) {
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0B1);
}

void enable_pwm_IN2_left(void) {
    TCCR1A |= (1 << COM1A1);
}

void disable_pwm_IN2_left(void) {
    TCCR1A &= ~(1 << COM1A1);
}

void enable_pwm_IN2_right(void) {
    TCCR2A |= (1 << COM2B1);
}

void disable_pwm_IN2_right(void) {
    TCCR2A &= ~(1 << COM2B1);
}

// ----- MOTOR CONTROL -----
void forward_right(uint8_t speed) {
    disable_pwm_IN2_right();
    DDRD |= (1 << PD3);
    PORTD &= ~(1 << PD3);
    
    TCCR0A |= (1 << COM0A1);
    OCR0A = speed;
}

void reverse_right(uint8_t speed) {
    TCCR0A &= ~(1 << COM0A1);
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);
    
    enable_pwm_IN2_right();
    OCR2B = speed;
}

void brake_right(void) {
    disable_pwm_IN1();
    disable_pwm_IN2_right();
    
    DDRD |= (1 << PD6) | (1 << PD3);
    PORTD |= (1 << PD6) | (1 << PD3);
}

void forward_left(uint8_t speed) {
    disable_pwm_IN2_left();
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);

    TCCR0A |= (1 << COM0B1);
    OCR0B = speed;
}

void reverse_left(uint8_t speed) {
    TCCR0A &= ~(1 << COM0B1);
    DDRD |= (1 << PD5);
    PORTD &= ~(1 << PD5);
    
    enable_pwm_IN2_left();
    OCR1A = speed;
}

void brake_left(void) {
    disable_pwm_IN1();
    disable_pwm_IN2_left();

    DDRD |= (1 << PD5);
    DDRB |= (1 << PB1);
    PORTD |= (1 << PD5);
    PORTB |= (1 << PB1);
}

// Combined motor control functions

void forward(uint8_t speed) {
    forward_left(speed);
    forward_right(speed);
}

void reverse(uint8_t speed) {
    reverse_left(speed);
    reverse_right(speed);
}

void brake(void) {
    brake_left();
    brake_right();
}

void cw_rotation(uint8_t speed) {
    forward_left(speed);
    reverse_right(speed);
}

void ccw_rotation(uint8_t speed) {
    forward_right(speed);
    reverse_left(speed);
}