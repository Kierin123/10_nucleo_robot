#include "stm32f1xx.h"
#include "core_cm3.h"
#include "motor.h"
#include "pwm_mk.h"
#include "GPIO_MK.h"

static motor_t motor_left;
static motor_t motor_right;

static void motor_f(motor_t motor, uint32_t direction)
{
    if ((direction == FORWARD_DIR) && (motor.control_v.state != MOTOR_STOP))
    {
        GPIO_write(motor.id_t, GPIOB, motor.config_v.pin_dir);
    }
    else
    {
        GPIO_write(!motor.id_t, GPIOB, motor.config_v.pin_dir);
    }
}

static void set_speed(uint32_t speed, uint8_t channel)
{

    TIM_pwm_set(MOTOR_PWM_TIMER, speed, channel); // set pwm value for Left motor
}

void motor_init()
{
    motor_left.control_v.speed = 0;
    motor_right.control_v.speed = 0;
    motor_left.control_v.direction = FORWARD_DIR;
    motor_right.control_v.direction = FORWARD_DIR;
    motor_left.control_v.state = MOTOR_STOP;
    motor_right.control_v.state = MOTOR_STOP;
    motor_left.config_v.pin_dir = PB1;
    motor_right.config_v.pin_dir = PB2;
    motor_left.config_v.pin_pwm = PB6;
    motor_right.config_v.pin_pwm = PB7;
    motor_right.control_v.aux_speed = 0;
    motor_left.control_v.aux_speed = 0;
    motor_left.id_t = 1;
    motor_right.id_t = 0;
    motor_left.channel = CH1;
    motor_right.channel = CH2;

    GPIO_pin_config(GPIOB, motor_left.config_v.pin_pwm, GPIO_ALTERNATE_PP_10HZ);  // wyjscie TIM3 CH1 kanał silnika
    GPIO_pin_config(GPIOB, motor_right.config_v.pin_pwm, GPIO_ALTERNATE_PP_10HZ); // wyjscie TIM3 CH2 kanał silnika

    GPIO_pin_config(GPIOB, motor_left.config_v.pin_dir, GPIO_OUTPUT_PP_10HZ);  // wyjscie DIR silnika
    GPIO_pin_config(GPIOB, motor_right.config_v.pin_dir, GPIO_OUTPUT_PP_10HZ); // wyjscie DIR silnika

    // SET TIMER for pwm control Channel 1 and Channel 2
    //______________________________________________________________________________

    TIM_setup(MOTOR_PWM_TIMER, pwm_mode, motor_left.channel);
    TIM_prescaler_set(MOTOR_PWM_TIMER, 5U);
    TIM_overload_set(MOTOR_PWM_TIMER, 500U);

    TIM_setup(MOTOR_PWM_TIMER, pwm_mode, motor_right.channel);
    TIM_prescaler_set(MOTOR_PWM_TIMER, 5U);
    TIM_overload_set(MOTOR_PWM_TIMER, 500U);

    //________________________________________________________________________________

    TIM_prescaler_set(MOTOR_SPEED_TIMER, 4999U);
    TIM_overload_set(MOTOR_SPEED_TIMER, 4U);
    TIM_setup_counter(MOTOR_SPEED_TIMER, up_count_mode);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    set_speed(motor_left.control_v.speed, motor_left.channel);
    set_speed(motor_right.control_v.speed, motor_right.channel);
}

void speed_up()
{
    if ((motor_left.control_v.speed >= 254) || (motor_right.control_v.speed >= 254))
    {
        motor_left.control_v.speed = 255;
        motor_right.control_v.speed = 255;
    }
    else
    {
        motor_left.control_v.speed += 2;
        motor_right.control_v.speed += 2;
        motor_left.control_v.aux_speed = motor_left.control_v.speed;
        motor_right.control_v.aux_speed = motor_right.control_v.speed;
    }
}

void speed_down()
{

    if ((motor_left.control_v.speed <= 0) || (motor_right.control_v.speed <= 0))
    {
        motor_left.control_v.speed = 0;
        motor_right.control_v.speed = 0;
    }
    else
    {
        motor_left.control_v.speed -= 2;
        motor_right.control_v.speed -= 2;
        motor_left.control_v.aux_speed = motor_left.control_v.speed;
        motor_right.control_v.aux_speed = motor_right.control_v.speed;
    }
}

void forward()
{
    motor_right.control_v.state = MOTOR_FORWARD;
    motor_left.control_v.state = MOTOR_FORWARD;
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;
    motor_f(motor_left, FORWARD_DIR);
    motor_f(motor_right, FORWARD_DIR);
}

void backward()
{
    stop();
    motor_right.control_v.state = MOTOR_BACKWARD;
    motor_left.control_v.state = MOTOR_BACKWARD;
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;
    motor_f(motor_left, BACKWARD_DIR);
    motor_f(motor_right, BACKWARD_DIR);
}

void forward_left()
{
    motor_right.control_v.state = MOTOR_FORWARD;
    motor_left.control_v.state = MOTOR_FORWARD;
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;

    motor_f(motor_right, FORWARD_DIR);
    motor_f(motor_left, FORWARD_DIR);

    motor_left.control_v.speed = motor_left.control_v.aux_speed / 2;
}

void forward_right()
{
    motor_right.control_v.state = MOTOR_FORWARD;
    motor_left.control_v.state = MOTOR_FORWARD;
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;

    motor_f(motor_right, FORWARD_DIR);
    motor_f(motor_left, FORWARD_DIR);

    motor_right.control_v.speed = motor_right.control_v.aux_speed / 2;
}

void turn_left()
{
    stop();
    motor_right.control_v.state = MOTOR_FORWARD;
    motor_left.control_v.state = MOTOR_BACKWARD;
    motor_f(motor_right, FORWARD_DIR);
    motor_f(motor_left, BACKWARD_DIR);
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;
}

void turn_right()
{
    stop();
    motor_right.control_v.state = MOTOR_BACKWARD;
    motor_left.control_v.state = MOTOR_FORWARD;
    motor_f(motor_right, BACKWARD_DIR);
    motor_f(motor_left, FORWARD_DIR);
    motor_right.control_v.speed = motor_right.control_v.aux_speed;
    motor_left.control_v.speed = motor_left.control_v.aux_speed;
}

void stop()
{
    motor_right.control_v.state = MOTOR_STOP;
    motor_left.control_v.state = MOTOR_STOP;
}

__attribute__((interrupt)) void TIM1_UP_IRQHandler(void)
{
    MOTOR_SPEED_TIMER->SR = ~TIM_SR_UIF;
    if ((motor_left.control_v.state != MOTOR_STOP) || (motor_right.control_v.state != MOTOR_STOP))
    {
        set_speed(motor_left.control_v.speed, motor_left.channel);
        set_speed(motor_right.control_v.speed, motor_right.channel);
    }
    else
    {
        set_speed(0, motor_left.channel);
        set_speed(0, motor_right.channel);
    }
}
