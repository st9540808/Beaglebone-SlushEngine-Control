#include <cstdio>
extern "C" { 
    #include <unistd.h>
    #include <sched.h>
}
#include <iostream>
#include <atomic>
#include <queue>
#include <mutex>
#include <cstring>
#include <ros/ros.h>
#include <motors/slushboard.h>
#include <motors/slushmotor.h>
#include <motors/gpio_pin.h>
#include <moveo_moveit/ArmJointState.h>
#include "mytimer.h"

/**
 * shoulder microsteps should not be less than 16
 */
#define SHOULDER_MICROSTEPS 2
#define remap_shoulder_steps(org) ((org) / (16 / SHOULDER_MICROSTEPS))

SlushBoard board;
SlushMotor axis0(0);
SlushMotor axis1_1(1);
SlushMotor axis1_2(2);
SlushMotor axis2(3);
SlushMotor axis3(4);
SlushMotor axis4(5);

std::atomic_bool joint_status(false); 
int position1, position2, position3, position4, position5;

void joint_steps_Callback(const moveo_moveit::ArmJointState& arm_steps)
{
    static int times = 1;
    ROS_INFO_NAMED("test", "times %d, arm_steps: %d %d %d %d %d %d", times++,
                   arm_steps.position1, arm_steps.position2, arm_steps.position3,
                   arm_steps.position4, arm_steps.position5, arm_steps.position6);
    
    position1 = arm_steps.position1;
    position2 = remap_shoulder_steps(arm_steps.position2);
    position3 = -arm_steps.position3;
    position4 = arm_steps.position4;
    position5 = arm_steps.position5;
    joint_status.store(true);
}

void move(void)
{
    axis0.goTo(position1);
    axis1_1.goTo(position2);
    axis1_2.goTo(-position2);
    axis2.goTo(position3);
    axis3.goTo(position4);
    axis4.goTo(position5);
}

void join(void)
{
    axis0.join();
    axis1_1.join();
    axis1_2.join();
    axis2.join();
    axis3.join();
    axis4.join();
}


int main(int argc, char* argv[])
{
    ROS_INFO("in main() function");
    
    struct sched_param schedp;
    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }

    axis0.resetDev();
    axis0.setMicroSteps(16);
    axis0.setMaxSpeed(150);
    axis1_1.resetDev();
    axis1_1.setMicroSteps(SHOULDER_MICROSTEPS);
    axis1_1.setMaxSpeed(35);
    axis1_2.resetDev();
    axis1_2.setMicroSteps(SHOULDER_MICROSTEPS);
    axis1_2.setMaxSpeed(35);
    axis2.resetDev();
    axis2.setMicroSteps(16);
    axis2.setMaxSpeed(200);
    axis3.resetDev();
    axis3.setMicroSteps(16);
    axis3.setMaxSpeed(45);
    axis4.resetDev();
    axis4.setMicroSteps(16);
    axis4.setMaxSpeed(200);

    axis1_1.setAccKVAL(73);
    axis1_1.setDecKVAL(73);
    axis1_1.setRunKVAL(65);
    axis1_1.setHoldKVAL(55);
    axis1_2.setAccKVAL(73);
    axis1_2.setDecKVAL(73);
    axis1_2.setRunKVAL(65);
    axis1_2.setHoldKVAL(55);
    axis1_1.setAcc(100);
    axis1_1.setDec(100);
    axis1_2.setAcc(100);
    axis1_2.setDec(100);

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_steps", 50, joint_steps_Callback);
    ros::Rate r(10);

    while (ros::ok()) {
        static int times = 0;

        if (joint_status.load()) {
            mytimer_start();
            move();
            mytimer_end();

            join();
            ROS_INFO_NAMED("test", "move() used %f secound\n", mytimer_diff());
            joint_status.store(false);
        }

        ros::spinOnce();
        r.sleep();
    }

    std::printf("exiting main");

    return 0;
}

