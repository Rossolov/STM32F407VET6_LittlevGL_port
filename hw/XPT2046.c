/*
 * XPT2046.c
 *
 *  Created on: 8 февр. 2020 г.
 *      Author: rsl_n
 */

#include "XPT2046.h"
//#include""


float ax = 1,  ay = 1;
int16_t bx = 300, by = 300;

int16_t calX[2], calY[2];

struct {
	uint16_t x, y, z;
}raw;

HAL_StatusTypeDef update();


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Init_XPT2046()
{
	update();
	Set_Touch_Cal_Point(0, CAL_X0, CAL_Y0);
	Set_Touch_Cal_Point(1, CAL_X1, CAL_Y1);
	HAL_Delay(1);
}

HAL_StatusTypeDef update()
{
		uint8_t pTxData[9] = {Z1,0,Z2,0,X,0,Y,0,0};
		uint8_t pRxData[9];
	    int16_t x, y, z1, z2;
	    HAL_StatusTypeDef result;

		result = HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, 9, 100);

		if(result == HAL_OK){
			z1 = ((pRxData[1]<<8) + pRxData[2])>>3;
			z2 = ((pRxData[3]<<8) + pRxData[4])>>3;
			x = ((pRxData[5]<<8) + pRxData[6])>>3;
			y = ((pRxData[7]<<8) + pRxData[8])>>3;
			if(!SWAP_XY){
				raw.x = x; raw.y = y;
			}
			else {
				raw.x = y; raw.y = x;
			}
			raw.z = 4095 + z1 - z2;
		}
	return result;
}

//return 1 in pressed
uint8_t Get_Touch_XY( volatile uint16_t *x_kor,volatile uint16_t *y_kor, uint8_t count_read, uint8_t calibration_flag)
{
	int32_t sumX = 0, sumY = 0;
	int16_t	num, touch_x, touch_y;
	num = count_read;

	for(int8_t i = num; i>0; i--){
		update();
		if(raw.x != 0 && raw.y != 0 && raw.z > Z_TH){
			sumX += raw.x;
			sumY += raw.y;
		}
		else num--;
	}
	if(num){
		touch_x = sumX / num;
		touch_y = sumY / num;
	}
	else return 0;

	//во время калибровки возвращаем вычисленные выше значения,
	//иначе производим расчёт координат, используя коэф. полученные при калибровке
	if (!calibration_flag)
	{
		*x_kor = map(touch_x, calX[0], calX[1], CAL_MARGIN, RES_X - CAL_MARGIN);
		*y_kor = map(touch_y, calY[0], calY[1], CAL_MARGIN, RES_Y - CAL_MARGIN);
	}
	else
	{
		*x_kor = touch_x;
		*y_kor = touch_y;
	}
	return 1;
}

uint8_t Get_Touch_Z(volatile uint16_t *z_kor){
	*z_kor = raw.z;
	return (raw.z > Z_TH);
}

void Set_Touch_Cal_Point(uint8_t pointNum, uint16_t x, uint16_t y){
	if(pointNum >= 2) return;
	calX[pointNum] = x;
	calY[pointNum] = y;
}

void Get_Touch_Cal_Point_Coord(uint8_t pointNum, uint16_t *x, uint16_t *y){
	if(pointNum == 0){
		*x = CAL_MARGIN;
		*y = CAL_MARGIN;
	}
	if(pointNum == 1){
		*x = RES_X - CAL_MARGIN;
		*y = RES_Y - CAL_MARGIN;
	}
}


