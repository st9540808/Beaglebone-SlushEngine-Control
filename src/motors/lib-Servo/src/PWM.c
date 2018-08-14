#include "PWM.h"

int PWM_EHRPWM1A_init(void)
{
    int fd;
    DIR* dir;
    
    if ((dir = opendir("/sys/class/pwm/pwmchip0/pwm-0:0")) != NULL) {
        closedir(dir);
        return 0;
    }

    fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY | O_APPEND);
    if (fd == -1) {
        perror("/sys/class/pwm/pwmchip0/export failed to open");
        return -1;
    }
    
    pwrite(fd, "0", 2, 0);
    close(fd);

    return 0;
}

int PWM_EHRPWM1A_set_period(long period)
{
    // the unit of period is nano second
    int fd, count, retry = 0, retryLimit = 1000;
    char str[15];
    
    if (period < 0) {
        fprintf(stderr, "period should not be less than 0\n");
        return -1;
    }

    // loop in case open() get permission error
    do {
        fd = open("/sys/class/pwm/pwmchip0/pwm-0:0/period", O_WRONLY | O_APPEND);
    } while (fd == -1 && ++retry != retryLimit && (usleep(1000), 1));
    
    if (retry == retryLimit) {
        perror("/sys/class/pwm/pwmchip0/pwm-0:0/period failed to open");
        return -1;
    }

    count = snprintf(str, 15, "%ld", period);
    pwrite(fd, str, count+1, 0);
    close(fd);
    
    return 0;
}

int PWM_EHRPWM1A_set_duty_cycle(long duty_cycle)
{
    // the unit duty_cycle is nano second
    int fd, count, retry = 0, retryLimit = 1000;
    char str[15];
    
    if (duty_cycle < 0) {
        fprintf(stderr, "duty_cycle should not be less than 0\n");
        return -1;
    }

    // loop in case open() get permission error
    do {
        fd = open("/sys/class/pwm/pwmchip0/pwm-0:0/duty_cycle", O_WRONLY | O_APPEND);
    } while (fd == -1 && ++retry != retryLimit && (usleep(1000), 1));
    
    if (retry == retryLimit) {
        perror("/sys/class/pwm/pwmchip0/pwm-0:0/duty_cycle failed to open");
        return -1;
    }

    count = snprintf(str, 15, "%ld", duty_cycle);
    pwrite(fd, str, count+1, 0);
    close(fd);
    
    return 0;
}

int PWM_EHRPWM1A_enable(void)
{
    int fd, count, retry = 0, retryLimit = 1000;

    do {
        fd = open("/sys/class/pwm/pwmchip0/pwm-0:0/enable", O_WRONLY | O_APPEND);
    } while (fd == -1 && ++retry != retryLimit && (usleep(1000), 1));
    
    if (retry == retryLimit) {
        perror("/sys/class/pwm/pwmchip0/pwm-0:0/enable failed to open");
        return -1;
    }

    pwrite(fd, "1", 2, 0);
    close(fd);
    
    return 0;
}

int PWM_EHRPWM1A_disable(void)
{
    int fd, count, retry = 0, retryLimit = 1000;

    do {
        fd = open("/sys/class/pwm/pwmchip0/pwm-0:0/enable", O_WRONLY | O_APPEND);
    } while (fd == -1 && ++retry != retryLimit && (usleep(1000), 1));
    
    if (retry == retryLimit) {
        perror("/sys/class/pwm/pwmchip0/pwm-0:0/enable (disable) failed to open");
        return -1;
    }

    pwrite(fd, "0", 2, 0);
    close(fd);
    
    return 0;
}

