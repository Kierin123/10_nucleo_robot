#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_SPEED_TIMER TIM1 // Define of the timer used for independent speed control
#define MOTOR_PWM_TIMER TIM4 // Define of the timer used for pwm control


 

typedef struct config
{
    uint32_t pin_dir;
    uint32_t pin_pwm;

} config_t;

typedef struct control
{
    uint32_t aux_speed;
    uint32_t speed;
    uint32_t direction;
    uint32_t state; // Forward, backward, stop

} control_t;

typedef struct motor
{
    control_t control_v;
    config_t config_v;
    uint32_t id_t;
    uint8_t channel;

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

void backward();

void turn_right();

void turn_left();

void forward_right();

void forward_left();

void stop();

#endif