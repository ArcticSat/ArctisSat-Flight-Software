#include "torque_rods.h"

void dipoleToPWM(float dipole, uint8_t * pwm, uint8_t * polarity)
{
    uint8_t calcPWM = MAX_PWM * abs(dipole / DIPOLE_SLOPE);

    if(calcPWM > MAX_PWM)
    {
        *pwm = MAX_PWM;
    }
    else
    {
        *pwm = calcPWM;
    }    

    if(dipole >= 0)
    {
        *polarity = 0;
    }
    else
    {
        *polarity = 1;
    }
}
