#ifndef SERVO_H
#define SERVO_H

#include <cassert>


class Servo {
    static constexpr double _1ms = 1000000;
    bool attached;

public:
    Servo();
    ~Servo();
    int attach();
    int detach();
    int write(int angle);
};

#endif

