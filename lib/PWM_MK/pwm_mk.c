#include "stm32f1xx.h"
#include "core_cm3.h"
#include "pwm_mk.h"

void TIM_setup(TIM_TypeDef *reg, TIM_mode_t mode, uint32_t channel_number)
{

    switch (mode)
    {
    case 1:
    {
        uint32_t volatile *var_reg;   // zmienna tymczasowa - rejestr do zapisania
        uint16_t ccmr1_reg_value = 0; // zmienna zapisu do rejestru ccmr1
        uint32_t reg_shift = (channel_number - 1) * 8;
        var_reg = &reg->CCER;
        *var_reg |= (1 << ((channel_number - 1) * 4));
        ccmr1_reg_value |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; //ustawienie wartosci rejestru konfiguracujnego ccmr1 dla pojedynczego kanalu

        if (channel_number >= 3)
        {
            var_reg = &reg->CCMR2;
            reg_shift -= 2;
        }
        else
        {
            var_reg = &reg->CCMR1;
        }

        *var_reg |= (ccmr1_reg_value << (reg_shift));

        // Basic registers setting to enable internal/external counting
        TIM_pwm_set(reg, 0, channel_number);
        reg->EGR = TIM_EGR_UG;
        reg->CR1 = TIM_CR1_CEN;
        reg->PSC = 1;    // default value
        reg->ARR = 1000; // default value
    }
    break;

    case 3:
    {
        reg->CCMR1 &= ~(TIM_CCMR1_IC1F);
        reg->CCMR1 |= (TIM_CCMR1_CC1S_1);
        reg->CCER |= (TIM_CCER_CC1P);
        reg->SMCR |= TIM_SMCR_TS_2;
        reg->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
        reg->DIER |= TIM_DIER_UIE;
        reg->CR1 |= TIM_CR1_CEN;
    }
    break;

    default:
        break;
    }
}

void TIM_setup_counter(TIM_TypeDef *reg, TIM_mode_t mode)
{
    switch (mode)
    {
    case 2:
    {
        reg->DIER |= TIM_DIER_UIE;
        reg->CR1 |= TIM_CR1_CEN;
    }
    break;
    case 4:
    {
        reg->CR1 |= TIM_CR1_DIR;
        reg->DIER |= TIM_DIER_UIE;
        reg->CR1 |= TIM_CR1_CEN;
    }
    break;
    }
}

void TIM_pwm_set(TIM_TypeDef *reg, uint32_t compare_value, const uint32_t channel_number)
{
    uint32_t volatile *var_reg;
    var_reg = &reg->CCR1;
    var_reg += (channel_number - 1);
    *var_reg = compare_value;
}

void TIM_prescaler_set(TIM_TypeDef *reg, uint32_t prescaler_value)
{
    reg->PSC = prescaler_value;
}

void TIM_overload_set(TIM_TypeDef *reg, uint32_t overload_value)
{
    reg->ARR = overload_value;
}
