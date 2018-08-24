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
    SlushMotor axis3(3);

    axis3.resetDev();
    axis3.hardStop();

    axis1.resetDev();
    axis1.setMicroSteps(4);
    axis1.setMaxSpeed(100);

    axis1.setAccKVAL(53);
    axis1.setDecKVAL(53);
    axis1.setRunKVAL(47);
    axis1.setAcc(300);
    axis1.setDec(300);
    std::printf("Hold: %d, Run: %d, Acc: %d, Dec: %d\n", axis1.getHoldKVAL(),
                axis1.getRunKVAL(), axis1.getAccKVAL(), axis1.getDecKVAL());
    std::printf("axis1.getAcc(): %f\n", axis1.getAcc());
    std::puts("");

    std::printf("move()\n");
    std::fflush(stdout);
   
	mytimer_start();
    axis1.goTo(-450);
    mytimer_end();
    std::printf("time: %f\n", mytimer_diff());

    axis1.join();
    sleep(2);
    
    axis1.goHome();
    axis1.join();
    sleep(1);

    return 0;
}

