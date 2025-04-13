#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor_control.h"
#include "spi.h"
#include "ultrasonic.h"
#include <util/delay.h>

int main(void) {
    spiInit();
    motorInit();
    // ultrasonicInit();
    // Since timer1 is used both for ultrasonic input capture and motor PWM, combined setup will be here
    DDRB |= (1 << L_R); // set left-reverse motor as output
    TCCR1A = (1 << WGM10) | (1 << COM1A1); // fast PWM, enables PWM for OC1A, clearing when match
    TCCR1 = (1 << WGM12) | (1 << CS11) | (1 << ICES1); // fast PWM, prescle by 8, and input capture set on rising edge 
    TIMSK1 = (1 << ICIE1); // enable input capture

    sei();

    while (1) {
        measureDistance();
        _delay_ms(100);
    }

    return 0;
}