#include "stm32f1xx.h"
#include "core_cm3.h"
#include "GPIO_MK.h"

void GPIO_pin_config(GPIO_TypeDef *port, Gpio_pin_t pin, Gpio_mode_t mode)
{
    uint32_t pin_v = __builtin_ctz(pin) * 4;

    uint32_t volatile *cr_reg;
    uint32_t cr_val;

    cr_reg = &port->CRL;

    if (pin_v > 28)
    {
        pin_v -= 32;
        cr_reg = &port->CRH;
    }
    cr_val = *cr_reg;
    cr_val &= ~(0x0f << pin_v);
    cr_val |= (mode << pin_v);

    *cr_reg = cr_val;
}

void GPIO_write(uint32_t pin_state, GPIO_TypeDef *port, Gpio_pin_t pin)
{
    uint32_t pin_v = __builtin_ctz(pin);
    uint32_t volatile *port_v;

    port_v = &port->ODR;

    if (pin_state)
    {
        *port_v |= (1 << pin_v);
    }
    else
    {
        *port_v &= ~(1 << pin_v);
    }
}

void GPIO_toggle(GPIO_TypeDef *port, Gpio_pin_t pin)
{

    uint32_t pin_v = __builtin_ctz(pin);
    uint32_t volatile *port_v;

    port_v = &port->ODR;

    *port_v ^= (1 << pin_v);
}

uint32_t GPIO_read(GPIO_TypeDef *port, Gpio_pin_t pin)
{
    uint32_t pin_v = __builtin_ctz(pin);
    uint32_t volatile *port_v;

    port_v = &port->IDR;

    if (*port_v & (1 << pin_v))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
