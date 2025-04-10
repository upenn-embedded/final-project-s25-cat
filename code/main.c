#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor_control.h"
#include "spi.h"
#include "ultrasonic.h"
#include <util/delay.h>

int main(void) {
    spiInit();
    motorInit();
    ultrasonicInit();

    sei();

    while (1) {}

    return 0;
}