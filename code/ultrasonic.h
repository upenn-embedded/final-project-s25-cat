//original ultrasonic.h

#include <stdint.h>

// #define TRIG_PIN PD4
// #define ECHO_PIN PB0

// void measureDistance();
// uint8_t getDistance();


//POLLING ultrasonic.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <avr/io.h>

#define TRIG_PIN PD4
#define ECHO_PIN PE2  // Changed from PB0 to PE2 (ICP3)

// void ultrasonic_init();
void measureDistance();
uint8_t getDistance();
void ultrasonic_timer3_init();

#endif
