#include "dotmatrix_8x8.h"

extern TIM_HandleTypeDef htim6;
uint8_t dotmatix_buf[8];

void data_dotmatrix(uint8_t data){
	for(uint8_t i=0;i<8;i++){
		col_dt((data>>i)&1);
		col_clk(1);col_clk(0);
	}
	col_stb(1);col_stb(0);
}

void dotmatix_display(void){
	row_rst(1);row_rst(0);
	for(uint8_t i=0;i<8;i++){
		row_clk(1);row_clk(0);
		data_dotmatrix(dotmatix_buf[i]);
		data_dotmatrix(0x00);
	}
}

void dotmatrix_put(uint8_t *data){
	for(uint8_t i=0;i<8;i++){
		dotmatix_buf[i]=data[i];
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if((htim->Instance)==TIM6){
		dotmatix_display();
	}
}
