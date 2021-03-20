#include "stm32f1xx.h"
#include "core_cm3.h"
#include "command.h"
#include "motor.h"



typedef void (*command_func_t)();   // Function pointer to point the function of the motor.h library. 

uint32_t active_command_flag = 0;   // Aux variable for remember active command in run

struct command
{
    uint8_t command_id;
    command_func_t action;
};

const struct command command_mapper[9] =
    {
        {FOR, forward},
        {BACK, backward},
        {FORWARD_LEFT, forward_left},
        {FORWARD_RIGHT, forward_right},
        {TURN_LEFT, turn_left},
        {TURN_RIGHT, turn_right},
        {STOP, stop},
        {SETSPEED_PLUS, speed_up},
        {SETSPEED_MINUS, speed_down}

};

void command_execute(command_t command)
{

    int i;
    for (i = 0; i <= 8; i++)
    {
        if (command == command_mapper[i].command_id)
        {
            command_mapper[i].action();
            active_command_flag = command_mapper[i].command_id;
        }
    }
}
