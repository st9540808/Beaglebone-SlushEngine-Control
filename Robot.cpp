#include <iostream>
#include "slushboard.h"
#include "slushmotor.h"

void say_hello() {
    std::cout << "Hello, from Robot!\n";
}

int main(int argc, char const *argv[])
{
    SlushBoard board;
    SlushMotor motor(0);

    return 0;
}
