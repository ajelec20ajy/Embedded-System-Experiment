/*
 * ADXL345.c
 *
 *  Created on: Jan 3, 2026
 *      Author: ajy97
 */
#include <ADXL345.h>
#include <math.h>
#define M_PI 3.14159265358979323846

uint8_t accel_buf[6];
static float ax_offset = 0.0f;
static float ay_offset = 0.0f;
static float az_offset = 0.0f;

static float filtered_ax = 0.0f;
static float filtered_ay = 0.0f;
static float filtered_az = 1.0f; // 초기 상태는 중력가속도 1g 가정
static const float alpha = 0.2f;

float ax, ay, az;


void Init_Accel(void) {

	  uint8_t val;
	  // 측정모드
	  val = 0x00; // 0000 1000 -> Measur만 킴.
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_POWER_CTL, 1, &val, 1, 100);
	  HAL_Delay(10);

	  val = 0x08; // 0000 1000 -> Measur만 킴.
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_POWER_CTL, 1, &val, 1, 100);
	  HAL_Delay(10);

	  // 데이터포맷 +-2g, FULL-RES
	  val = 0x08;
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_DATA_FORMAT, 1, &val, 1, 100);
	  HAL_Delay(10);

	  // 100Hz - 0x0A
	  val = 0x09; //09면 25Hz -> 1/25 = 40ms
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_BW_RATE, 1, &val, 1, 100);
	  HAL_Delay(10);

	  // 인터럽트핀 = INT1
	  val = 0x00;
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_INT_MAP, 1, &val, 1, 100);
	  HAL_Delay(10);

	  // 인터럽트 Enable
	  val = 0x80;
	  HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL_INT_ENABLE, 1, &val, 1, 100);
	  HAL_Delay(10);


    // 오프셋 캘리브레이션
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int SAMPLES = 1000;
    for (int i = 0; i < 500; i++) { // 초기값 그냥 버리기
        HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, accel_buf, 6, HAL_MAX_DELAY);
        HAL_Delay(1);
    }

    // 오프셋 캘리브레이션
    for (int i = 0; i < SAMPLES; i++) { // 오프셋 측정
        HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_DATAX0,
                         I2C_MEMADD_SIZE_8BIT, accel_buf, 6, HAL_MAX_DELAY);
        int16_t rx = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
        int16_t ry = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
        int16_t rz = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);

        ax_sum += rx;
        ay_sum += ry;
        az_sum += rz;
        HAL_Delay(1);
    }

    // 오프셋 캘리브레이션
    ax_offset = ax_sum / (float)SAMPLES;
    ay_offset = ay_sum / (float)SAMPLES;
    az_offset = (az_sum / (float)SAMPLES) - 256.0f;
}

//DMA 기반 읽기
void Read_Accel(void) {
    HAL_I2C_Mem_Read_DMA(&hi2c1, ADXL345_I2C_ADDR, ADXL_REG_DATAX0,
                         I2C_MEMADD_SIZE_8BIT, accel_buf, 6);
}

//DMA 완료 처리
void Accel_ProcessData(void) {
    int16_t raw_ax = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
    int16_t raw_ay = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
    int16_t raw_az = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);

    //4 mg/LSB
    float scale_g_per_lsb = 0.004f;

    // 오프셋 보정 + g 변환
    ax = (raw_ax - ax_offset) * scale_g_per_lsb;
    ay = (raw_ay - ay_offset) * scale_g_per_lsb;
    az = (raw_az - az_offset) * scale_g_per_lsb;

    // EMA : 이번엔 안걸음. 왜냐? 빠르게 반응하기 위해서죠
    /*
    filtered_ax = (alpha * ax_g) + (1.0f - alpha) * filtered_ax;
    filtered_ay = (alpha * ay_g) + (1.0f - alpha) * filtered_ay;
    filtered_az = (alpha * az_g) + (1.0f - alpha) * filtered_az;

    ax = filtered_ax;
    ay = filtered_ay;
    az = filtered_az;
    */
}
