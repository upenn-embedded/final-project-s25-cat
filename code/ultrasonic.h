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

#define TRIG_PORT PORTD
#define TRIG_BIT  PD4

#define ECHO_BIT     PB0  
void ultrasonic_init();
void measureDistance();
uint8_t getDistance();

#endif
