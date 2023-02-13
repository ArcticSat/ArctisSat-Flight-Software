#include <stdint.h>

#define DIPOLE_SLOPE 0.04

#define MAX_PWM 255.0

void dipoleToPWM(float dipole, uint8_t * pwm, uint8_t * polarity);
