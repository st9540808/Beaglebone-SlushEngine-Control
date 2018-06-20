#include <cstdio>
extern "C" { 
    #include <unistd.h>
}
#include "slushboard.h"
#include "slushmotor.h"
#include "gpio_pin.h"

int main(int argc, char const *argv[])
{
    const int stepmove = 300000;// #the number of steps we want to be moving
    SlushBoard board;
    SlushMotor axis1(0);


    axis1.resetDev();

    // #move the motor in one direction and wait for it to finish
    while (axis1.isBusy()) ;
    axis1.move(stepmove);
    while (axis1.isBusy()) ;
    axis1.move(-stepmove);
    // #when these operations are finished shut off the motor
    while (axis1.isBusy()) ;

    axis1.free();
    return 0;
}
