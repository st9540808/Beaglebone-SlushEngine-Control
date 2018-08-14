#ifndef PWM_H
#define PWM_H

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

int PWM_EHRPWM1A_init(void);
int PWM_EHRPWM1A_set_period(long period);
int PWM_EHRPWM1A_set_duty_cycle(long duty_cycle);
int PWM_EHRPWM1A_enable(void);
int PWM_EHRPWM1A_disable(void);

#endif
