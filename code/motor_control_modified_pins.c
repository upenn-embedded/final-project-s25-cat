#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "motor_control_modified_pins.h"

void motorInit() {
    timer0_init();
    timer1_init();
    timer2_init();
}

// TODO: fine tune speed and
// ----- TIMER INITIALIZATIONS -----
//
// Timer0 is used for forward channels on both motors
// Timer1 is used for left motor reverse (OC1A on PB1)
// Timer2 is used for right motor reverse (OC2B on PD3)

void timer0_init(void) {
    // Set PD5 and PD6 as outputs (used for left and right forward channels)
    DDRD |= (1 << PD5) | (1 << PD6);
    
    // Fast PWM, 8-bit mode using TCCR0A (WGM00 and WGM01)
    TCCR0A = (1 << WGM00) | (1 << WGM01);
    
    // Set prescaler = 8 with TCCR0B (CS01)
    TCCR0B = (1 << CS01);
}

void timer1_init(void) {
    // Use PB1 for left motor reverse (OC1A)
    DDRB |= (1 << PB1);
    
    // Fast PWM, 8-bit mode (WGM10 in TCCR1A and WGM12 in TCCR1B)
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler = 8
}

void timer2_init(void) {
    // Use PD3 for right motor reverse (OC2B)
    DDRD |= (1 << PD3);
    
    // Fast PWM, 8-bit mode for Timer2 (WGM20 and WGM21)
    TCCR2A = (1 << WGM20) | (1 << WGM21);
    
    // Set prescaler = 8 (CS21)
    TCCR2B = (1 << CS21);
    
    // Clear output compare for OC2B initially
    OCR2B = 0;
}

// ----- PWM CONTROL HELPERS -----
//
// The following helper functions enable or disable the PWM output channels.
// Note that the forward channels are on Timer0 (PD5, PD6) and the reverse
// channels use Timer1 (left motor on PB1) and Timer2 (right motor on PD3).

void enable_pwm_IN1(void) {
    // Right motor forward (PD6) using Timer0 channel A
    TCCR0A |= (1 << COM0A1);
    // Left motor forward (PD5) using Timer0 channel B
    TCCR0A |= (1 << COM0B1);
}

void disable_pwm_IN1(void) {
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0B1);
}

void enable_pwm_IN2_left(void) {
    // Enable PWM on left motor reverse (PB1) using Timer1 channel A
    TCCR1A |= (1 << COM1A1);
}

void disable_pwm_IN2_left(void) {
    TCCR1A &= ~(1 << COM1A1);
}

void enable_pwm_IN2_right(void) {
    // Enable PWM on right motor reverse (PD3) using Timer2 channel B
    TCCR2A |= (1 << COM2B1);
}

void disable_pwm_IN2_right(void) {
    TCCR2A &= ~(1 << COM2B1);
}

// ----- MOTOR CONTROL FUNCTIONS -----
//
// The motor connections now are defined as follows:
//
// RIGHT MOTOR:
//   Forward: IN1 on PD6 (Timer0 OC0A)
//   Reverse: IN2 on PD3 (Timer2 OC2B)
//
// LEFT MOTOR:
//   Forward: IN1 on PD5 (Timer0 OC0B)
//   Reverse: IN2 on PB1 (Timer1 OC1A)

void forward_right(uint8_t speed) {
    // First, disable the reverse channel on PD3
    disable_pwm_IN2_right();
    DDRD |= (1 << PD3);
    PORTD &= ~(1 << PD3);
    
    // Enable PWM for forward channel on PD6 (Timer0 channel A)
    TCCR0A |= (1 << COM0A1);
    OCR0A = speed;
}

void reverse_right(uint8_t speed) {
    // Disable forward PWM on PD6
    TCCR0A &= ~(1 << COM0A1);
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);
    
    // Enable PWM for reverse channel on PD3 (Timer2 channel B)
    enable_pwm_IN2_right();
    OCR2B = speed;
}

void brake_right(void) {
    // Turn off both forward and reverse PWM outputs for the right motor.
    disable_pwm_IN1();
    disable_pwm_IN2_right();
    
    // Set PD6 and PD3 as outputs and drive them high to brake.
    DDRD |= (1 << PD6) | (1 << PD3);
    PORTD |= (1 << PD6) | (1 << PD3);
}

void forward_left(uint8_t speed) {
    // Disable reverse PWM on left motor (PB1)
    disable_pwm_IN2_left();
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);
    
    // Enable PWM for forward channel on PD5 (Timer0 channel B)
    TCCR0A |= (1 << COM0B1);
    OCR0B = speed;
}

void reverse_left(uint8_t speed) {
    // Disable forward PWM on PD5
    TCCR0A &= ~(1 << COM0B1);
    DDRD |= (1 << PD5);
    PORTD &= ~(1 << PD5);
    
    // Enable PWM for reverse channel on PB1 (Timer1 channel A)
    enable_pwm_IN2_left();
    OCR1A = speed;
}

void brake_left(void) {
    disable_pwm_IN1();
    disable_pwm_IN2_left();
    
    // Set PD5 and PB1 as outputs and drive them high for brake.
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

void cw_rotation(uint8_t speed, uint16_t angle) {
    // TODO: actually determine angle
    // For a clockwise rotation, drive the left motor forward and the right motor in reverse.
    forward_left(speed);
    reverse_right(speed);
    _delay_ms(20000);  // Fixed delay as in the original code.
}

// int main(void) {
//     timer0_init();
//     timer1_init();
//     timer2_init();
    
//     // Enable global interrupts if needed
//     // sei();
    
//     while (1) {
//         cw_rotation(100, 90);
//         _delay_ms(20000);
//     }
    
//     return 0;
// }
