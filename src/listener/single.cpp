#include <cstdio>
extern "C" { 
    #include <unistd.h>
    #include <time.h>
    #include <sched.h>
}
#include <vector>
#include <algorithm>
#include <motors/slushboard.h>
#include <motors/slushmotor.h>
#include <motors/gpio_pin.h>
#include "mytimer.h"

int main(int argc, char const *argv[])
{
    const int stepmove = 400; // #the number of steps we want to be moving
    struct sched_param schedp;

    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }
    
    SlushBoard board;
    SlushMotor axis1(4);

    axis1.resetDev();
    axis1.setMicroSteps(16);
    axis1.setMaxSpeed(60);

    std::printf("isBusy()\n");
    std::printf("axis1.isBusy(): %d\n", axis1.isBusy());
    std::puts("");

    std::printf("move()\n");
    std::fflush(stdout);
   
	mytimer_start();
    axis1.goTo(750);
    mytimer_end();
    std::printf("time: %f\n", mytimer_diff());

    axis1.join();
    sleep(1);
    
    axis1.goHome();
    axis1.join();
    sleep(1);

    return 0;
}
