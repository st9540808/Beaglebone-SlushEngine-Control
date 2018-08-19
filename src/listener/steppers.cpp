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

std::vector<std::vector<int>> joint_steps_queue = {
    {7, 7, -16, 9, 4, 0},
    {64},
    {121},
    {186},
    {326},
    {502},
    {718},
    {970},
    {1257},
    {1577},
    {1928},
    {2306},
    {2711},
    {3124},
    {3537},
    {3950},
    {4363},
    {4776},
    {5189},
    {5602},
    {6001},
    {6716},
    {7027},
    {7304},
    {7549},
    {7753},
    {7918},
    {8041},
    {8098},
    {8155},
    {8194},
    {8300},
    {0}
};

int main(int argc, char const *argv[])
{
    const int stepmove = 400; // #the number of steps we want to be moving
    struct sched_param schedp;

    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }
    
    SlushBoard board;
    SlushMotor axis0(0);
    SlushMotor axis1(1);
    SlushMotor axis1_2(2);
    SlushMotor axis2(3);
    SlushMotor axis3(4);
    SlushMotor axis4(5);

    axis0.resetDev();
    axis0.setMicroSteps(16);
    axis0.setMaxSpeed(300);
    axis1.resetDev();
    axis1.setMicroSteps(16);
    axis1.setMaxSpeed(50);
    axis1_2.resetDev();
    axis1_2.setMicroSteps(16);
    axis1_2.setMaxSpeed(50);
    axis2.resetDev();
    axis2.setMicroSteps(16);
    axis2.setMaxSpeed(750);
    axis3.resetDev();
    axis3.setMicroSteps(16);
    axis3.setMaxSpeed(45);
    axis4.resetDev();
    axis4.setMicroSteps(16);
    axis4.setMaxSpeed(300);

    // #move the motor in one direction and wait for it to finish
    std::printf("move()\n");
    std::fflush(stdout);
   
	mytimer_start();
    axis0.goTo(2500);
    axis1.goTo(1500);
    axis1_2.goTo(-1500);
    axis2.goTo(-20000);
    axis3.goTo(-800);
    axis4.goTo(1500);
    mytimer_end();
    std::printf("time: %f\n", mytimer_diff());

    sleep(4);
    axis0.join();
    axis1.join();
    axis1_2.join();
    axis2.join();
    axis3.join();
    axis4.join();
    sleep(1);

    axis0.goHome();
    axis1.goHome();
    axis1_2.goHome();
    axis2.goHome();
    axis3.goHome();
    axis4.goHome();
    
    sleep(4);
    axis0.join();
    axis1.join();
    axis1_2.join();
    axis2.join();
    axis3.join();
    axis4.join();
    sleep(1);
    /*
	for (auto joint_steps : joint_steps_queue) {
	    mytimer_start();
        axis1.goTo(joint_steps[0]);
        mytimer_end();
        std::printf("position %4d, time: %f\n", joint_steps[0], mytimer_diff());
        
        axis1.join();
        sleep(1);
    }
    */

    // when these operations are finished shut off the motor
    return 0;
}
