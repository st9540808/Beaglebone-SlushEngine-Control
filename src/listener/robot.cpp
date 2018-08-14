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

SlushBoard board;
SlushMotor axis1(0);

std::atomic_bool joint_status(false); 
int position1, position2, position3, position4, position5;

void joint_steps_Callback(const moveo_moveit::ArmJointState& arm_steps)
{
    static int times = 1;
    ROS_INFO_NAMED("test", "times %d, arm_steps: %d %d %d %d %d %d", times++,
                   arm_steps.position1, arm_steps.position2, arm_steps.position3,
                   arm_steps.position4, arm_steps.position5, arm_steps.position6);
    
    position1 = arm_steps.position1;
    position2 = arm_steps.position2;
    position3 = arm_steps.position3;
    position4 = arm_steps.position4;
    position5 = arm_steps.position5;
    joint_status.store(true);
}

int main(int argc, char* argv[])
{
    ROS_INFO("in main() function");
    
    struct sched_param schedp;
    moveo_moveit::ArmJointState joint_steps;
    
    schedp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        perror("sched_setscheduler");
    }

    axis1.resetDev();
    axis1.setMicroSteps(16);
    axis1.setMaxSpeed(200);

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_steps", 50, joint_steps_Callback);
    ros::Rate r(10);

    while (ros::ok()) {
        static int times = 0;

        if (joint_status.load()) {
            mytimer_start();
            axis1.goTo(position1);
            mytimer_end();
            axis1.join();
            
            ROS_INFO_NAMED("test", "axis1.goTo(%d) used %f secound\n",
                           position1, mytimer_diff());
            joint_status.store(false);
        }

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("exiting main");

    return 0;
}

