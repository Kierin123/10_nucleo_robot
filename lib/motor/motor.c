#include "stm32f1xx.h"
#include "core_cm3.h"
#include "motor.h"
#include "pwm_mk.h"
#include "GPIO_MK.h"

static uint32_t aux_speed_left = 0;
static uint32_t aux_speed_right = 0;

static motor_t motor_left;
static motor_t motor_right;

static void motor_left_f(uint32_t direction)
{
    if ((direction == FORWARD_DIR) && (motor_left.state != MOTOR_STOP))
    {
        GPIO_write(1, GPIOB, motor_left.pin_dir);
    }
    else
    {
        GPIO_write(0, GPIOB, motor_left.pin_dir);
    }
}

static void motor_right_f(uint32_t direction)
{
    if ((direction == FORWARD_DIR) && (motor_right.state != MOTOR_STOP))
    {
        GPIO_write(0, GPIOB, motor_right.pin_dir);
    }
    else
    {
        GPIO_write(1, GPIOB, motor_right.pin_dir);
    }
}

static void set_speed_left_motor(uint32_t speed)
{
    TIM_pwm_set(TIM3, speed, CH1); // set pwm value for Left motor
}

static void set_speed_right_motor(uint32_t speed)
{
    TIM_pwm_set(TIM3, speed, CH2); // set pwm value for Right motor
}

void motor_init()
{
    motor_left.speed = 0;
    motor_right.speed = 0;
    motor_left.direction = FORWARD_DIR;
    motor_right.direction = FORWARD_DIR;
    motor_left.state = MOTOR_STOP;
    motor_right.state = MOTOR_STOP;
    motor_left.pin_dir = PB1;
    motor_right.pin_dir = PB2;
    motor_left.pin_pwm = PA6;
    motor_right.pin_pwm = PA7;

    GPIO_pin_config(GPIOA, motor_left.pin_pwm, GPIO_ALTERNATE_PP_10HZ);  // wyjscie TIM3 CH1 kanał silnika
    GPIO_pin_config(GPIOA, motor_right.pin_pwm, GPIO_ALTERNATE_PP_10HZ); // wyjscie TIM3 CH2 kanał silnika

    GPIO_pin_config(GPIOB, motor_left.pin_dir, GPIO_OUTPUT_PP_10HZ);  // wyjscie DIR silnika
    GPIO_pin_config(GPIOB, motor_right.pin_dir, GPIO_OUTPUT_PP_10HZ); // wyjscie DIR silnika

    TIM_prescaler_set(TIM4, 4999U);
    TIM_overload_set(TIM4, 4U);
    TIM_setup_counter(TIM4, up_count_mode);
    NVIC_EnableIRQ(TIM4_IRQn);

    set_speed_left_motor(motor_left.speed);
    set_speed_right_motor(motor_right.speed);
}

void speed_up()
{
    if ((motor_left.speed >= 254) || (motor_right.speed >= 254))
    {
        motor_left.speed = 255;
        motor_right.speed = 255;
    }
    else
    {
        motor_left.speed += 2;
        motor_right.speed += 2;
        aux_speed_left = motor_left.speed;
        aux_speed_right = motor_right.speed;
    }
}

void speed_down()
{

    if ((motor_left.speed <= 0) || (motor_right.speed <= 0))
    {
        motor_left.speed = 0;
        motor_right.speed = 0;
    }
    else
    {
        motor_left.speed -= 2;
        motor_right.speed -= 2;
        aux_speed_left = motor_left.speed;
        aux_speed_right = motor_right.speed;
    }
}

void forward()
{
    motor_right.state = MOTOR_FORWARD;
    motor_left.state = MOTOR_FORWARD;
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;
    motor_right_f(FORWARD_DIR);
    motor_left_f(FORWARD_DIR);
}

void backward()
{
    stop();
    motor_right.state = MOTOR_BACKWARD;
    motor_left.state = MOTOR_BACKWARD;
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;
    motor_right_f(BACKWARD_DIR);
    motor_left_f(BACKWARD_DIR);
}

void forward_left()
{
    motor_right.state = MOTOR_FORWARD;
    motor_left.state = MOTOR_FORWARD;
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;

    motor_right_f(FORWARD_DIR);
    motor_left_f(FORWARD_DIR);

    motor_left.speed = motor_left.speed / 2;
}

void forward_right()
{
    motor_right.state = MOTOR_FORWARD;
    motor_left.state = MOTOR_FORWARD;
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;

    motor_right_f(FORWARD_DIR);
    motor_left_f(FORWARD_DIR);

    motor_right.speed = motor_right.speed / 2;
}

void turn_left()
{
    stop();
    motor_right.state = MOTOR_FORWARD;
    motor_left.state = MOTOR_BACKWARD;
    motor_right_f(FORWARD_DIR);
    motor_left_f(BACKWARD_DIR);
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;
}

void turn_right()
{
    stop();
    motor_right.state = MOTOR_BACKWARD;
    motor_left.state = MOTOR_FORWARD;
    motor_right_f(BACKWARD_DIR);
    motor_left_f(FORWARD_DIR);
    motor_right.speed = aux_speed_right;
    motor_left.speed = aux_speed_left;
}

void stop()
{
    motor_right.state = MOTOR_STOP;
    motor_left.state = MOTOR_STOP;
}

__attribute__((interrupt)) void TIM4_IRQHandler(void)
{
    TIM4->SR = ~TIM_SR_UIF;
    if ((motor_left.state != MOTOR_STOP) || (motor_right.state != MOTOR_STOP))
    {
        set_speed_left_motor(motor_left.speed);
        set_speed_right_motor(motor_right.speed);
    }
    else
    {
        set_speed_left_motor(0);
        set_speed_right_motor(0);
    }
}
