
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


// ----- INIT PWM TIMERS -----


// === TIMER INITIALIZATION ===

void timer0_init() {
    DDRD |= (1 << PD5) | (1 << PD6);

    // Fast PWM, 8-bit (WGM01 + WGM00)
    TCCR0A = (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS01); // Prescaler = 8
}

void timer1_init() {
    DDRB |= (1 << PB1) | (1 << PB2);

    // Fast PWM 8-bit (WGM13:0 = 0b0100)
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
}

// === PWM CONTROL HELPERS ===

void enable_pwm_IN1() {
    TCCR0A |= (1 << COM0A1); // PD6 (OC0A) - Right IN1
    TCCR1A |= (1 << COM1A1); // PB1 (OC1A) - Left IN1 (swapped)
}

void disable_pwm_IN1() {
    TCCR0A &= ~(1 << COM0A1);
    TCCR1A &= ~(1 << COM1A1);
}

void enable_pwm_IN2() {
    TCCR1A |= (1 << COM1B1); // PB2 (OC1B) - Right IN2
    TCCR0A |= (1 << COM0B1); // PD5 (OC0B) - Left IN2 (swapped)
}

void disable_pwm_IN2() {
    TCCR1A &= ~(1 << COM1B1);
    TCCR0A &= ~(1 << COM0B1);
}

// === RIGHT MOTOR ===

void forward_right(uint8_t speed) {
    TCCR1A &= ~(1 << COM1B1); // Disable PWM on IN2 (PB2)
    DDRB |= (1 << PB2);
    PORTB &= ~(1 << PB2);

    TCCR0A |= (1 << COM0A1); // Enable PWM on IN1 (PD6)
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

// === LEFT MOTOR (IN1 = PD5, IN2 = PB1 after swap) ===

void forward_left(uint8_t speed) {
    TCCR1A &= ~(1 << COM1B1); // Disable PWM on IN2 (PB1)
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);

    TCCR0A |= (1 << COM0B1); // Enable PWM on IN1 (PD5)
    OCR0B = speed;
}

void reverse_left(uint8_t speed) {
    TCCR0A &= ~(1 << COM0B1);
    DDRD |= (1 << PD5);
    PORTD &= ~(1 << PD5);

    TCCR1A |= (1 << COM1A1); // Enable PWM on IN2 (PB1)
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

void cw_rotation(uint8_t speed, uint16_t angle) {
    // Forward left and reverse right
    // Untested, might miraculously actually work but probably not
    
    forward_left(speed);
    _delay_ms(20000);
}

int main(void) {
    timer0_init();
    timer1_init();

    while (1) {
        //add delay after moving motor, only then break
        /*
         brake();
         _delay_ms(20000);
         forward_left(1000);
         _delay_ms(20000);
         brake();
         forward_right(1000);
         _delay_ms(20000);
         brake();
         reverse_left(1000);
         _delay_ms(20000);
         brake();
         reverse_right(1000);
         _delay_ms(20000);
         brake();
         
         //both
         forward(1000);
         _delay_ms(20000);
         brake();
         reverse(1000);
         
         //different speeds
         _delay_ms(20000);
         brake();
         forward(100);
         _delay_ms(20000);
         brake();
         reverse(100);
         _delay_ms(20000);
*/

         // cw_rotation
         cw_rotation(100,90);
         _delay_ms(20000);
         
         
    }
}