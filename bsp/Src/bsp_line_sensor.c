#include "bsp_line_sensor.h"
#include "main.h"

/** line_sensor_binding_t：一条 OUT 信号对应的 GPIO 端口和引脚。 */
typedef struct {
    GPIO_TypeDef *gpio_port; /* CubeMX 生成的 GPIO 端口。 */
    uint16_t gpio_pin;       /* CubeMX 生成的引脚位掩码。 */
} line_sensor_binding_t;

static const line_sensor_binding_t line_sensor_bindings[LINE_SENSOR_COUNT] = {
    {LINE_OUT1_GPIO_Port, LINE_OUT1_Pin}, {LINE_OUT2_GPIO_Port, LINE_OUT2_Pin},
    {LINE_OUT3_GPIO_Port, LINE_OUT3_Pin}, {LINE_OUT4_GPIO_Port, LINE_OUT4_Pin},
    {LINE_OUT5_GPIO_Port, LINE_OUT5_Pin},
};

uint8_t bsp_line_sensor_read_raw(void) {
    uint8_t raw_levels = 0U; /* 打包后的五路原始电平。 */
    for (uint8_t sensor_index = 0U; sensor_index < LINE_SENSOR_COUNT; ++sensor_index) {
        const line_sensor_binding_t *binding = &line_sensor_bindings[sensor_index];
        if (HAL_GPIO_ReadPin(binding->gpio_port, binding->gpio_pin) == GPIO_PIN_SET) {
            raw_levels |= (uint8_t)(1U << sensor_index);
        }
    }
    return raw_levels;
}
