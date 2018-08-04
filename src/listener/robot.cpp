#include <cstdio>
extern "C" { 
    #include <unistd.h>
}
#include <ros/ros.h>
#include <motors/slushboard.h>
#include <motors/slushmotor.h>
#include <motors/gpio_pin.h>
#include <moveo_moveit/ArmJointState.h>

void test_motors(void)
{
    const int stepmove = 400; // #the number of steps we want to be moving
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
    axis1.goTo(800);
    std::puts("");
    
    while (axis1.isBusy()) ;
    sleep(1);

    std::printf("move()\n");
    axis1.goHome();
    std::puts("");
    
    // #when these operations are finished shut off the motor
    while (axis1.isBusy()) ;

    axis1.free();
}

void joint_steps_Callback(const moveo_moveit::ArmJointState& arm_steps)
{
    static int times = 0;
    ROS_INFO_NAMED("test", "Received times: %d", times++);
    ROS_INFO_NAMED("test", "arm_steps: %d %d %d %d %d %d",
                   arm_steps.position1, arm_steps.position2, arm_steps.position3,
                   arm_steps.position4, arm_steps.position5, arm_steps.position6);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_steps", 50, joint_steps_Callback);

    ros::spin();

    return 0;
}
