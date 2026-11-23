#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

/* 只模拟本模块读取的 GPIO 输入寄存器，不模拟实际电气电平及波形。 */
typedef struct {
    uint32_t IDR;
} GPIO_TypeDef;

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef test_gpio_b;
#define GPIOB (&test_gpio_b)
#define LINE_OUT1_Pin (1U << 12)
#define LINE_OUT2_Pin (1U << 13)
#define LINE_OUT3_Pin (1U << 14)
#define LINE_OUT4_Pin (1U << 15)
#define LINE_OUT5_Pin (1U << 3)
#define LINE_OUT1_GPIO_Port GPIOB
#define LINE_OUT2_GPIO_Port GPIOB
#define LINE_OUT3_GPIO_Port GPIOB
#define LINE_OUT4_GPIO_Port GPIOB
#define LINE_OUT5_GPIO_Port GPIOB

/**
 * HAL_GPIO_ReadPin：返回测试寄存器指定引脚的状态。
 * gpio_port：端口；gpio_pin：引脚掩码。返回：模拟高低电平。
 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

#endif
