#include <cstdio>
extern "C" { 
    #include <unistd.h>
}
#include "slushboard.h"
#include "slushmotor.h"
#include "gpio_pin.h"

int main(int argc, char const *argv[])
{
    GPIO_Pin pin;
    pin.init(P8_15);

    for (int i = 0; i < 20; i++) {
        pin.set();
        usleep(12000);
        pin.clear();
        usleep(12000);
    }

    return 0;
}
