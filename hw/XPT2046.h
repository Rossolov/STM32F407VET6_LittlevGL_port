/*
 * XPT2046.h
 *
 *  Created on: 8 февр. 2020 г.
 *      Author: rsl_n
 */

#ifndef HW_XPT2046_H_
#define HW_XPT2046_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define Z_TH 500
#define CAL_MARGIN 20
#define RES_X	320
#define RES_Y	240
#define SWAP_XY 1

#define CAL_X0 400
#define CAL_Y0 400
#define CAL_X1 3500
#define CAL_Y1 3500

extern SPI_HandleTypeDef hspi2;

#define	Y 	0x90
#define	X 	0xD0
#define Z1 	0xB0
#define Z2	0xC0

void Init_XPT2046();
uint8_t Spi_Master_Transmit(uint8_t out_data);
uint8_t Get_Touch_XY( volatile uint16_t *x_kor,volatile uint16_t *y_kor, uint8_t count_read, uint8_t calibration_flag);
uint8_t Get_Touch_Z(volatile uint16_t *z_kor);
void Get_Touch_Cal_Point_Coord(uint8_t pointNum, uint16_t *x, uint16_t *y);
void Set_Touch_Cal_Point(uint8_t pointNum, uint16_t x, uint16_t y);

#endif /* HW_XPT2046_H_ */
