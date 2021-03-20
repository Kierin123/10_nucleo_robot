#ifndef PWM_MK_H_
#define PWM_MK_H_

enum
{
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,

};

typedef enum
{
    pwm_mode = 1,
    up_count_mode = 2,
    down_count_mode = 4,
    ext_count_mode = 3,

} TIM_mode_t;



void TIM_setup(TIM_TypeDef *reg, TIM_mode_t mode, uint32_t channel_number);

void TIM_setup_counter(TIM_TypeDef *reg, TIM_mode_t mode);

void TIM_pwm_set(TIM_TypeDef *reg, uint32_t compare_value, uint32_t channel_number);

void TIM_prescaler_set(TIM_TypeDef *reg, uint32_t prescaler_value);

void TIM_overload_set(TIM_TypeDef *reg, uint32_t overload_value);

#endif