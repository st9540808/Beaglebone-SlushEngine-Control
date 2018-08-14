extern "C" {
    #include <PWM.h>
}
#include "Servo.h"

Servo::Servo()
    : attached(false)
{
    PWM_EHRPWM1A_init();
    PWM_EHRPWM1A_set_period(20000000);
}

Servo::~Servo()
{
    PWM_EHRPWM1A_disable();
}

int Servo::attach()
{
    this->attached = true;
    return PWM_EHRPWM1A_enable();
}

int Servo::detach()
{
    this->attached = false;
    return PWM_EHRPWM1A_disable();
}

int Servo::write(int angle)
{
    assert(0 <= angle && angle <= 180); 

    int ret = 0;
    int duty_cycle =
        500000 + 2 * _1ms * (static_cast<double>(angle) / 180);       
    
    if (!this->attached)
        ret = this->attach();
    ret = PWM_EHRPWM1A_set_duty_cycle(duty_cycle);

    return ret;
}

