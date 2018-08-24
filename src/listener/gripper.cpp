#include <motors/Servo.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    Servo gripper;
    
    gripper.write(45);
    sleep(1);
    gripper.detach();
    sleep(1);

    gripper.write(68);
    sleep(1);
    gripper.detach();
    sleep(1);

    gripper.setOrigin();
    sleep(1);

    return 0;
}
