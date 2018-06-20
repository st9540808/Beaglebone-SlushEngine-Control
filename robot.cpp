#include <cstdio>
extern "C" { 
    #include <unistd.h>
}
#include "slushboard.h"
#include "slushmotor.h"
#include "gpio_pin.h"

int main(int argc, char const *argv[])
{
    SlushBoard board;
    SlushMotor axis(0);

    return 0;
}
