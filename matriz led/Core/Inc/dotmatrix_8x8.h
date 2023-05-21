#ifndef __DOTMATRIX_8X8_H
#define __DOTMATRIX_8X8_H

#include "stm32l0xx_hal.h"

#define row_rst(x)	x?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define row_clk(x)	x?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)

#define col_dt(x) 	x?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET)
#define col_clk(x)	x?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET)
#define col_stb(x)	x?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET)

void data_dotmatrix(uint8_t data);
void dotmatix_display(void);
void dotmatrix_put(uint8_t *data);

#endif
