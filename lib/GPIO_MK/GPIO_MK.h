#ifndef GPIO_MK_H_
#define GPIO_MK_H_


typedef enum
{
    GPIO_OUTPUT_PP_10HZ = 1,
    GPIO_OUTPUT_PP_2HZ = 2,
    GPIO_OUTPUT_PP_50HZ = 3,

    GPIO_OUTPUT_OPEN_10HZ = 4,
    GPIO_OUTPUT_OPEN_2HZ = 5,
    GPIO_OUTPUT_OPEN_50HZ = 7,

    GPIO_ALTERNATE_PP_10HZ = 9,
    GPIO_ALTERNATE_PP_2HZ = 10,
    GPIO_ALTERNATE_PP_50HZ = 11,

    GPIO_ALTERNATE_OPEN_10HZ = 13,
    GPIO_ALTERNATE_OPEN_2HZ = 14,
    GPIO_ALTERNATE_OPEN_50HZ = 15,

    GPIO_INPUT_FLOW = 4,

    GPIO_INPUT_PULL = 8,

} Gpio_mode_t;

typedef enum
{
    PB0 = 0x00000001,
    PB1 = 0x00000002,
    PB2 = 0x00000004,
    PB3 = 0x00000008,
    PB4 = 0x00000010,
    PB5 = 0x00000020,
    PB6 = 0x00000040,
    PB7 = 0x00000080,
    PB8 = 0x00000100,
    PB9 = 0x00000200,
    PB10 = 0x00000400,
    PB11 = 0x00000800,
    PB12 = 0x00001000,
    PB13 = 0x00002000,
    PB14 = 0x00004000,
    PB15 = 0x00008000,

    PA0 = 0x00000001,
    PA1 = 0x00000002,
    PA2 = 0x00000004,
    PA3 = 0x00000008,
    PA4 = 0x00000010,
    PA5 = 0x00000020,
    PA6 = 0x00000040,
    PA7 = 0x00000080,
    PA8 = 0x00000100,
    PA9 = 0x00000200,
    PA10 = 0x00000400,
    PA11 = 0x00000800,
    PA12 = 0x00001000,
    PA13 = 0x00002000,
    PA14 = 0x00004000,
    PA15 = 0x00008000,

    PC0 = 0x00000001,
    PC1 = 0x00000002,
    PC2 = 0x00000004,
    PC3 = 0x00000008,
    PC4 = 0x00000010,
    PC5 = 0x00000020,
    PC6 = 0x00000040,
    PC7 = 0x00000080,
    PC8 = 0x00000100,
    PC9 = 0x00000200,
    PC10 = 0x00000400,
    PC11 = 0x00000800,
    PC12 = 0x00001000,
    PC13 = 0x00002000,
    PC14 = 0x00004000,
    PC15 = 0x00008000,

    PD0 = 0x00000001,
    PD1 = 0x00000002,
    PD2 = 0x00000004,
    PD3 = 0x00000008,
    PD4 = 0x00000010,
    PD5 = 0x00000020,
    PD6 = 0x00000040,
    PD7 = 0x00000080,
    PD8 = 0x00000100,
    PD9 = 0x00000200,
    PD10 = 0x00000400,
    PD11 = 0x00000800,
    PD12 = 0x00001000,
    PD13 = 0x00002000,
    PD14 = 0x00004000,
    PD15 = 0x00008000,

} Gpio_pin_t;


void GPIO_pin_config(GPIO_TypeDef *port, Gpio_pin_t pin, Gpio_mode_t mode);

void GPIO_write(uint32_t pin_state, GPIO_TypeDef *port, Gpio_pin_t pin);

void GPIO_toggle(GPIO_TypeDef *port, Gpio_pin_t pin);

uint32_t GPIO_read(GPIO_TypeDef *port, Gpio_pin_t pin);


#endif