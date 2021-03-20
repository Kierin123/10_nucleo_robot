#ifndef COMMAND_H_
#define COMMAND_H_

// _______ Command library to setup the mappers for remote control of the robot ____


// ______ Move commands for the Robot _______
enum
{
    FOR = 97,
    BACK = 98,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
    SETSPEED_PLUS,
    SETSPEED_MINUS,

};

typedef uint32_t command_t;

void command_execute(command_t command);

#endif