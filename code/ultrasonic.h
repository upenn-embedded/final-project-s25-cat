#include <stdint.h>

#define TRIG_PIN PD4
#define ECHO_PIN PB0 // TODO: check that the input capture actually works with timer 3...
// TODO: has anyone actually used timer 3? do we really need to be multiplexing aaaaaaaaa

void ultrasonicInit();
uint16_t measureDistance();