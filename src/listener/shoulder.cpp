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
    const int steps = -500;
    struct sched_param schedp;

    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }
    
    SlushBoard board;
    SlushMotor axis1(1);
    SlushMotor axis2(2);
    SlushMotor axis3(3);

    axis1.resetDev();
    axis1.setMicroSteps(2);
    axis1.setMaxSpeed(40);
    axis2.resetDev();
    axis2.setMicroSteps(2);
    axis2.setMaxSpeed(40);

    axis1.hardStop();
    axis2.hardStop();

    axis3.resetDev();
    axis3.setMicroSteps(16);
    axis3.setMaxSpeed(180);

    axis1.setAccKVAL(73);
    axis1.setDecKVAL(73);
    axis1.setRunKVAL(65);
    axis1.setHoldKVAL(55);
    axis2.setAccKVAL(73);
    axis2.setDecKVAL(73);
    axis2.setRunKVAL(65);
    axis2.setHoldKVAL(55);
    axis1.setAcc(100);
    axis1.setDec(100);
    axis2.setAcc(100);
    axis2.setDec(100);

    std::printf("Hold: %d, Run: %d, Acc: %d, Dec: %d\n", axis1.getHoldKVAL(),
                axis1.getRunKVAL(), axis1.getAccKVAL(), axis1.getDecKVAL());
    std::printf("axis1.getAcc(): %f\n", axis1.getAcc());
    std::puts("");

    std::printf("move()\n");
    std::fflush(stdout);
   
	mytimer_start();
    axis1.goTo(steps);
    axis2.goTo(-steps);
    mytimer_end();
    std::printf("time: %f\n", mytimer_diff());

    axis3.goTo(-11000);
    axis1.join();
    axis2.join();
    axis3.join();
    sleep(2);
    
    axis1.goHome();
    axis2.goHome();
    axis3.goHome();
    axis1.join();
    axis2.join();
    axis3.join();
    sleep(1);
    

    return 0;
}

