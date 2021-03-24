#include "config.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include "core_cm3.h"
#include "GPIO_MK.h"
#include "pwm_mk.h"
#include "motor.h"
#include "uart_mk.h"
#include "command.h"

// ______ Aux variables ___________________

volatile uint32_t R_blink_count = 0;
volatile uint32_t pwm_psc_value = 0;
uint32_t pwm_flag_1 = 1;
uint32_t tmp = 0;

static volatile uint32_t Tim2_encoder[9];
static volatile uint32_t Tim2_index;

static volatile uint32_t Tim3_encoder[9];
static volatile uint32_t Tim3_index;

int main(void)
{
    // Alternative function pins enable and config
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_0;

    // Clocks enabling
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN; // I/O B, I/O A, TIM1 Enable
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;                      // TIM2, TIM3 Clock Enable
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                                           //
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    GPIO_pin_config(GPIOA, PA5, GPIO_OUTPUT_PP_10HZ); // Build_In led output

    //
    // Konfiguracja TIM2 jako wejście enkodera
    //______________________________________________________________________________

    GPIO_write(1, GPIOA, PA0);
    GPIO_write(1, GPIOA, PA1);

    GPIO_pin_config(GPIOA, PA0, GPIO_INPUT_PULL); // wyjscie TIM2 CH1 kanał enkodera
    GPIO_pin_config(GPIOA, PA1, GPIO_INPUT_PULL); // wyjscie TIM2 CH2 kanał enkodera

    TIM2->SMCR |= TIM_SMCR_SMS_1;

    TIM2->CCER |= TIM_CCER_CC1P;
    TIM2->ARR |= 720;

    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_CMS_1 | TIM_CR1_CMS_0;
    TIM2->CNT = 0x7FFF;

    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);

    //
    // Konfiguracja TIM3 jako wejście enkodera
    //______________________________________________________________________________

    GPIO_write(1, GPIOA, PA6);
    GPIO_write(1, GPIOA, PA7);

    GPIO_pin_config(GPIOA, PA6, GPIO_INPUT_PULL); // wyjscie TIM3 CH1 kanał enkodera
    GPIO_pin_config(GPIOA, PA7, GPIO_INPUT_PULL); // wyjscie TIM3 CH2 kanał enkodera

    TIM3->SMCR |= TIM_SMCR_SMS_1;

    TIM3->CCER |= TIM_CCER_CC1P;
    TIM3->ARR |= 720;

    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_CMS_1 | TIM_CR1_CMS_0;
    TIM3->CNT = 0x7FFF;

    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);

    // Motor Init
    //______________________________________________________________________________

    motor_init();

    // UART1 Init

    uart1_init(9600);



    SysTick_Config(8000000UL * 0.005);

    while (1)
    {
        tmp = get_from_buffer();
        if (tmp != 0)
        {
            command_execute(tmp);
            // printf("command received!");
        }
    }
}

__attribute__((interrupt)) void SysTick_Handler(void)
{

    Tim2_encoder[Tim2_index] = TIM2->CNT;
    Tim2_index++;
    if (Tim2_index > 9)
    {
        Tim2_index = 0;
    }

    Tim3_encoder[Tim3_index] = TIM3->CNT;
    Tim3_index++;
    if (Tim3_index > 9)
    {
        Tim3_index = 0;
    }

    //GPIO_toggle(GPIOA, PA5);
}

__attribute__((interrupt)) void TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_UIF;
    //TIM2->CNT = 0;
    //GPIO_toggle(GPIOA, PA5);
}

__attribute__((interrupt)) void TIM3_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;
    //TIM1->CNT = 0;
    // GPIO_toggle(GPIOA, PA5);
}
