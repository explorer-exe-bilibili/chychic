#ifndef BSP_LINE_SENSOR_H
#define BSP_LINE_SENSOR_H

#include <stdint.h>

#define LINE_SENSOR_COUNT 5U   /* HW-871 的数字输出数量。 */
#define LINE_SENSOR_MASK 0x1FU /* 五路有效位，bit0～bit4 对应 OUT1～OUT5。 */

/**
 * bsp_line_sensor_read_raw：读取五路原始电平，打包为一个位图。
 * 参数：无。返回：bit0～bit4 依次对应 OUT1～OUT5，1 为高电平，0 为低电平。
 * 调用前须由 MX_GPIO_Init 配置输入和 No Pull；本函数不更改引脚模式。
 * 此处不解释黑白或左右顺序，交给应用层按实物配置归一化。
 */
uint8_t bsp_line_sensor_read_raw(void);

#endif
