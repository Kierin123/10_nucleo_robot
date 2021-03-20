#ifndef MOTOR_H_
#define MOTOR_H_

typedef struct motor
{
    uint32_t speed;
    uint32_t direction;
    uint32_t state;
    uint32_t pin_dir;
    uint32_t pin_pwm;

} motor_t;

enum
{
    FORWARD_DIR,
    BACKWARD_DIR,

};

enum motor_state
{
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
};



void motor_init();

void speed_up();

void speed_down();

void forward();

void backward( );

void turn_right( );

void turn_left( );

void forward_right( );

void forward_left( );

void stop( );


#endif