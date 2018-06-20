#include <cstdio>
extern "C" { 
    #include <unistd.h>
}
#include "slushboard.h"
#include "slushmotor.h"
#include "gpio_pin.h"

int main(int argc, char const *argv[])
{
    const int stepmove = 1000;// #the number of steps we want to be moving
    SlushBoard board;
    SlushMotor axis1(0);

    axis1.resetDev();
    axis1.setMicroSteps(2);
    axis1.setMaxSpeed(200);

    // #move the motor in one direction and wait for it to finish
    std::printf("isBusy()\n");
    std::printf("%d\n", axis1.isBusy());
    std::puts("");

    std::printf("move()\n");
    axis1.move(stepmove);
    std::puts("");
    
    while (axis1.isBusy()) ;

    std::printf("move()\n");
    axis1.move(-stepmove);
    std::puts("");
    
    // #when these operations are finished shut off the motor
    while (axis1.isBusy()) ;

    axis1.free();
    return 0;
}
