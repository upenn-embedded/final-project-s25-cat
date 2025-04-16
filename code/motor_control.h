#include <stdint.h>

#define R_F PD6
#define R_R PD3
#define L_F PD5
#define L_R PB1

void motorInit();

void timer0_init(void);
void timer2_init(void);

void enable_pwm_IN1(void);
void disable_pwm_IN1(void);
void enable_pwm_IN2_left(void);
void disable_pwm_IN2_left(void);
void enable_pwm_IN2_right(void);
void disable_pwm_IN2_right(void);

void forward_right(uint8_t speed);
void reverse_right(uint8_t speed);
void brake_right(void);
void forward_left(uint8_t speed);
void reverse_left(uint8_t speed);
void brake_left(void);
void forward(uint8_t speed);
void reverse(uint8_t speed);
void brake(void);
void cw_rotation(uint8_t speed);
void ccw_rotation(uint8_t speed);