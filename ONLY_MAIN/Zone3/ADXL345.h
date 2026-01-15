/*
 * ADXL345.h
 *
 *  Created on: Jan 3, 2026
 *      Author: ajy97
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

typedef struct {
	uint64_t at[2];
    int16_t x;
    int16_t y;
    int16_t z;
} Data_Accel_ADXL;

extern Data_Accel_ADXL adxlData;
extern uint8_t accel_buf[6];
extern float ax;
extern float ay;
extern float az;

// 기본 I2C 7-bit 주소: ALT_ADDR 핀이 HIGH일 때 0x1D, LOW일 때 0x53.
// 보드(모듈) 연결 상태에 맞춰 바꿔 사용.
// (데이터시트: ALT ADDRESS 관련 설명 및 주소 표). :contentReference[oaicite:1]{index=1}
#define ADXL345_I2C_ADDR     0x53<<1  // 7-bit address (change to 0x53 if ALT grounded)

// ADXL345 레지스터 (필요한 것들만)
#define ADXL_REG_DEVID       0x00
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_BW_RATE     0x2C
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATAX0      0x32
#define ADXL_REG_OFSTX       0x1E
#define ADXL_REG_OFSTY       0x1F
#define ADXL_REG_OFSTZ       0x20
#define ADXL_REG_FIFO_CTL    0x38
#define ADXL_INT_ENABLE  0x2E
#define ADXL_INT_MAP     0x2F
#define ADXL_INT_SOURCE  0x30
// 함수
void Init_Accel(void);
void Read_Accel(void);
void Accel_ProcessData(void);

#endif /* INC_ADXL345_H_ */
