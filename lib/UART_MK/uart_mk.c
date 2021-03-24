#include "stm32f1xx.h"
#include "core_cm3.h"
#include "uart_mk.h"
#include "GPIO_MK.h"
#include <time.h>

#define RX_PIN PA10
#define TX_PIN PA9



// _______ RX variables ________________________

uint32_t RX_buffer[32] = {0}; // circular register RX
uint32_t Rx_buffer_head = 0;  // RX buffer head pointer
uint32_t Rx_buffer_tail = 0;  // RX buffer tail pointer
uint32_t Rx_buffer_empty = 0;

// _______ TX variables ________________________

uint32_t TX_buffer[32] = {0}; // circular register RX
uint32_t Tx_buffer_head = 0;  // RX buffer head pointer
uint32_t Tx_buffer_tail = 0;  // RX buffer tail pointer
uint32_t Tx_buffer_empty = 0;






void uart1_init(uint32_t baud_rate)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // GPIO_pin_config(GPIOA, PA9, GPIO_ALTERNATE_PP_2HZ); // TX output
    GPIO_pin_config(GPIOB, TX_PIN, GPIO_ALTERNATE_PP_2HZ); // TX output
    GPIO_pin_config(GPIOA, RX_PIN, GPIO_INPUT_FLOW);      // RX Input

    USART1->BRR = (8000000UL / baud_rate);
    USART1->CR1 |= USART_CR1_UE;
    USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;

    NVIC_EnableIRQ(USART1_IRQn);
}



uint32_t get_from_buffer()
{
    uint32_t temp_char = 0;
    if (Rx_buffer_tail == Rx_buffer_head)
    {
        Rx_buffer_empty = 1;
        return 0;
    }
    else
    {
        temp_char = RX_buffer[Rx_buffer_tail];
        Rx_buffer_tail++;
        if (Rx_buffer_tail > 32)
            Rx_buffer_tail = 0;
        return temp_char;
    }
}



__attribute__((interrupt)) void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
    {
        USART1->SR &= ~USART_SR_RXNE;
        uint32_t temp_head;
        if (Rx_buffer_head == Rx_buffer_tail)
        {
            // Error handling
        }
        temp_head = Rx_buffer_head;
        RX_buffer[temp_head] = USART1->DR;
        Rx_buffer_head++;
        if (Rx_buffer_head > 32)
            Rx_buffer_head = 0;

        //USART1->DR = tmp;
    }
}


