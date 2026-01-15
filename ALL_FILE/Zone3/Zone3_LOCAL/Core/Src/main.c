
#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;

/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LED */
osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Accel */
osThreadId_t AccelHandle;
const osThreadAttr_t Accel_attributes = {
  .name = "Accel",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void canTask(void *argument);
void ledTask(void *argument);
void accelTask(void *argument);

// led 동작은 Zone3_LED의 main.c 참고
#include "ADXL345.h"
#include "string.h" //memcpy
#include "stdio.h" // snprinf 로깅용

#define LED_OFF		0xFF // LED 모두 꺼라
#define LED_READY		0x01 // 환영등(Zone 4의 IR 감지 시): LED1 ON, LED2 OFF
#define LED_EMERGENCY 	0x02 // 비상등(Zone 1, 2의 충돌 감지 시): LED1 OFF, LED2 ON
#define LED_CENTER_DOWN	0x04 // 두 LED 모두 깜빡여서 중앙 Down 알림
#define LED_BROKEN_SELF 0x08 // 파란 LED 깜빡여서 스스로의 CAN 고장을 알림

#define FLAG_STATE_OFF			0x11
#define FLAG_STATE_READY		0x22
#define FLAG_STATE_DRIVE		0x44
#define FLAG_STATE_EMERGENCY	0x88

#define FLAG_SPI_DMA_COMPLETE		0x1111
#define FLAG_I2C_DMA_COMPLETE		0x2222


static uint8_t crc_xor(const uint8_t *p, int n) { // CRC 만드는 함수
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= p[i];
  return c;
}

#pragma pack(push,1)
typedef struct{
	uint8_t header;
	int8_t data;
	uint8_t status;
	uint8_t seq;
	uint16_t reserved;
	uint8_t crc;
	uint8_t tail;
}SPI_Packet_t;
#pragma pack(pop)

typedef union{
	SPI_Packet_t pkt;
	uint8_t bytes[8];
}SPI_Buffer_t;

volatile uint8_t cnt = 0;

volatile uint8_t SPI1_ERR_FLAG = 0;
volatile uint8_t I2C1_ERR_FLAG = 0;
volatile uint8_t I2C1_ERR_CNT = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) // 사용 중인 SPI 확인
    {
    	osThreadFlagsSet(LEDHandle, FLAG_SPI_DMA_COMPLETE);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1)
    {
		HAL_SPI_DMAStop(hspi); // 멈추고
		hspi->State = HAL_SPI_STATE_READY; // 강제로 통신 가능 상태로 복구
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	I2C1_ERR_CNT = I2C1_ERR_FLAG = 0;
        osThreadFlagsSet(AccelHandle, FLAG_I2C_DMA_COMPLETE);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) { // I2C1 포트에서 에러 발생 시. SPI랑 다르게 I2C는 ACK 체킹 가능!
    	if(++I2C1_ERR_CNT>5){
    		I2C1_ERR_FLAG = 1;
    	}
    }
}
// CAN
#define OFF 0x01
#define READY 0x02
#define DRIVE 0x04
#define EMERGENCY 0x08
#define MOTOR_ERR 0x10
#define PEDAL_ERR 0x20
#define CENTRAL_DOWN 0x40
#define BROKEN_SELF 0x80

#define ID_1_SONIC 0x010
#define ID_1_PEDAL 0x040
#define ID_1_HB 0x071
#define ID_1_PEDAL_SPI_ERR 0x022

#define ID_2_SONIC 0x011
#define ID_2_MOTOR_STATUS 0x110
#define ID_2_MOTOR_SPI_ERR 0x020
#define ID_2_HB 0x072

#define ID_3_ACC 0x012
#define ID_3_HB 0x073

#define ID_4_IR 0x142
#define ID_4_BTN 0x144
#define ID_4_HB 0x074

#define ID_CENTRAL_MOTOR_ERR 0x001
#define ID_CENTRAL_EMERGENCY 0x002
#define ID_CENTRAL_STATUS 0x004 // = HEARBEAT ID OF CENTRAL
#define ID_HARDWARE_ERR 0x7FF
// Zone1에 설명 있음
typedef enum { // 에러 종류
    ERR_TYPE_RX_STUCK,
    ERR_TYPE_RX_TIMEOUT,
    ERR_TYPE_TX_FAIL,
    ERR_TYPE_HW,
    ERR_TYPE_CHKSUM
} CAN_Err_Type;
// Zone1에 설명 있음
typedef union {
    struct {
        uint16_t data;
        uint8_t reserved[4];
        uint8_t counter;
        uint8_t checksum;
    } __attribute__((packed)) data16;
    struct {
        uint16_t data1;
        uint16_t data2;
        uint8_t reserved[2];
        uint8_t counter;
        uint8_t checksum;
    } __attribute__((packed)) data32;
    struct {
        int16_t ax;
        int16_t ay;
        int16_t az;
    	uint8_t counter;
    	uint8_t checksum;
        } __attribute__((packed)) accel_data;
    uint8_t bytes[8];
} CAN_Payload_t; // CAN PACKET

typedef struct {
    int16_t x, y, z;
} AccelData_t;

AccelData_t g_accel_raw; // 공유 자원 -> 보호 필요

typedef struct{
	uint8_t z1;
	uint8_t z2;
	uint8_t z3;
	uint8_t z4;
	uint8_t central;
} CAN_CNT; // 다용도 카운터

// Zone1에 설명 있음
CAN_CNT stuck_counter;
CAN_CNT last_counter;
CAN_CNT can_rx_err_cnt;
volatile uint32_t last_zone_rx_time;
volatile uint8_t state_global = OFF;
uint8_t global_tx_counter = 0;

// Zone1에 설명 있음
void CAN_OFF_FILTER(){
	CAN_FilterTypeDef canFilter;
	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter.FilterIdHigh = (0x001 << 5);
	canFilter.FilterIdLow = (0x002 << 5);
	canFilter.FilterMaskIdHigh = (0x004 << 5);
	canFilter.FilterMaskIdLow = (0x000 << 5);
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterActivation = DISABLE;
	canFilter.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan, &canFilter);
}

// Zone1에 설명 있음
void CAN_ERR_HANDLE(uint32_t ID, CAN_Err_Type type){ //송,수신 포괄 에러 처리
	switch(ID){
	case ID_3_ACC:
		can_rx_err_cnt.z3++;
		if(can_rx_err_cnt.z3>=20){
			state_global = BROKEN_SELF;
		}
		break;

	case ID_CENTRAL_STATUS:
		can_rx_err_cnt.central++;
		if(can_rx_err_cnt.central>=20){
			CAN_OFF_FILTER();
			state_global = CENTRAL_DOWN;
			HAL_CAN_Stop(&hcan);
		}
		break;

	case ID_HARDWARE_ERR:
		if(type == ERR_TYPE_HW){
			state_global = BROKEN_SELF;

		}
		break;

	default:
		break;
	}
}

// Zone1에 설명 있음
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    CAN_Payload_t payload = {0};
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, payload.bytes) == HAL_OK){
    	if(rxHeader.IDE == CAN_ID_STD){
    		uint8_t received_sum = 0;
			for(int i = 0; i<7; i++){ // Checksum
				received_sum += payload.bytes[i];
			}
			if(received_sum != payload.bytes[7]) {
				CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_CHKSUM);
				return; // 이번거 데이터 오염되었다 판단, 그냥 건너뜀
			}
			// 이제 데이터 읽음
			switch(rxHeader.StdId){
			case ID_CENTRAL_MOTOR_ERR:
				if (payload.data16.counter == last_counter.central) {
					if (++stuck_counter.central > 5) {
						CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_RX_STUCK);
						return; // 처리 후 return
					}
				}
				else {
					stuck_counter.central = 0;
					last_counter.central = payload.data16.counter;
				}

				state_global = MOTOR_ERR; // state_global은 8bit인데, 8bit 대입은 atomic하게 처리된다. 별도의 cs나 락필요없
				can_rx_err_cnt.central = 0;
				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
				break;

			case ID_CENTRAL_EMERGENCY:
				if (payload.data16.counter == last_counter.central) {
					if (++stuck_counter.central > 5) {
						CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_RX_STUCK);
						return; // 처리 후 return
					}
				}
				else {
					stuck_counter.central = 0;
					last_counter.central = payload.data16.counter;
				}

				state_global = EMERGENCY;
				can_rx_err_cnt.central = 0;
				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
				break;

			case ID_CENTRAL_STATUS:
				if(payload.data16.counter == last_counter.central){
					if(++stuck_counter.central > 5){
					CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_RX_STUCK);
					return; // 처리 후 return
					}
				}
				else {
					stuck_counter.central = 0;
					last_counter.central = payload.data16.counter;
				}

				if(rxHeader.DLC == 8){
					state_global = payload.data16.data;
				}
				can_rx_err_cnt.central = 0;
				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지
				break;
			}
    	}
    }
}


void CAN_INIT() { // Zone1에 설명 있음
	CAN_FilterTypeDef canFilter;

	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter.FilterIdHigh = (0x001 << 5);
	canFilter.FilterIdLow = (0x002 << 5);
	canFilter.FilterMaskIdHigh = (0x004 << 5);
	canFilter.FilterMaskIdLow = (0x000 << 5);
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterActivation = ENABLE;
	canFilter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
		Error_Handler();
	}

	last_zone_rx_time = HAL_GetTick();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) { // Zone1에 설명 있음
	      uint32_t err = HAL_CAN_GetError(hcan);
	      uint8_t any = 0;

	      if(err & HAL_CAN_ERROR_BOF){
	    	  any = 1;
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW);
	      }

	      if(err & HAL_CAN_ERROR_ACK){
	    	  any = 1;
	    	  can_rx_err_cnt.central++;
	      }

	      if(err & HAL_CAN_ERROR_RX_FOV0){
	    	  any = 1;
	    	  CAN_RxHeaderTypeDef rxHeader;
	    	  uint8_t dummyData[8];

	    	  uint32_t fillLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

	    	  while (fillLevel > 0) {
	    	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, dummyData);
	    	  fillLevel--;
	    	  }
	      }
	      if(!any){ // 위의 경우들은 아닌경우, 포괄에러처리
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW);
	      }
}

void CAN_SEND(uint32_t ID) { // Zone1에 설명 있음
	    CAN_TxHeaderTypeDef txHeader ={0};
	    uint32_t txMailbox;
	    CAN_Payload_t payload ={0};
	    AccelData_t snap_accel;

	    txHeader.IDE = CAN_ID_STD;
	    txHeader.RTR = CAN_RTR_DATA;
	    txHeader.TransmitGlobalTime = DISABLE;
	    txHeader.StdId = ID;
	    txHeader.DLC = 8;

	    switch(ID){
		case ID_3_ACC:
			if(I2C1_ERR_FLAG){
				payload.accel_data.ax = payload.accel_data.ay = payload.accel_data.az = 0xFFFF;
			}
			taskENTER_CRITICAL();
				snap_accel = g_accel_raw;
			taskEXIT_CRITICAL();
			payload.accel_data.ax = snap_accel.x;
			payload.accel_data.ay = snap_accel.y;
			payload.accel_data.az = snap_accel.z;
			break;
		default: // heartbeat
			payload.data16.data = state_global;
	    }

	    payload.data16.counter = global_tx_counter++;
	    uint8_t sum = 0;
	        for (int i = 0; i < 7; i++) {
	        	sum += payload.bytes[i];
	        }
	        payload.data16.checksum = sum;

	    uint32_t tick = HAL_GetTick();
	    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
	    		CAN_ERR_HANDLE(ID, ERR_TYPE_TX_FAIL); // 메일박스 고임 타임아웃처리
	    		return; // 타임아웃 시 포기
	    }

	    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, payload.bytes, &txMailbox) != HAL_OK) {
	    	CAN_ERR_HANDLE(ID, ERR_TYPE_HW); // 창구에서 거절
	    }
}

static void Build_SPI_Packet(SPI_Buffer_t *buffer, uint8_t data){ // SPI 패킷 구성하는 함수
	uint8_t *b = buffer->bytes;

	b[0] = 0xAA;
	b[1] = data;
	b[2] = state_global;
	b[3] = cnt++;
	b[4] = 0x0; // reserved
	b[5] = 0x0; // reserved
	b[6] = crc_xor(b, 6);
	b[7] = 0xED;
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();

  Init_Accel();
  CAN_INIT();

  osKernelInitialize();

  CANHandle = osThreadNew(canTask, NULL, &CAN_attributes);
  LEDHandle = osThreadNew(ledTask, NULL, &LED_attributes);
  AccelHandle = osThreadNew(accelTask, NULL, &Accel_attributes);

  osKernelStart();

  while (1)
  {

  }
}


void canTask(void *argument)
{
  uint8_t state_local = OFF;
  uint32_t tick = osKernelGetTickCount();
  const uint32_t kPeriod = 10U; // 10ms 주기 (100Hz)
  for(;;)
  {
	  tick += kPeriod;
	  state_local = state_global;
	  if ((state_local != OFF) && (state_local != CENTRAL_DOWN) && (state_local != BROKEN_SELF))
	        {
	            if (HAL_GetTick() - last_zone_rx_time > 1000) {
	          	  state_local = state_global = CENTRAL_DOWN;

	            }
	  }
      switch(state_local){
      case OFF:
    	  break;

      case READY:
          CAN_SEND(ID_3_HB); // 하트비트 데이터로는 모터 PWM 상태 보냄
          break;

      case DRIVE:
          CAN_SEND(ID_3_HB); // 하트비트 데이터로는 모터 PWM 상태 보냄
          CAN_SEND(ID_3_ACC); // 가속도 전송 - GET // 만약 FLAG_I2C1_ERR가 떳다면 CAN_SEND내부에서 데이터를 0xFFFF로 가공한다.
          break;

      case PEDAL_ERR:
      case MOTOR_ERR:
      case EMERGENCY:
          CAN_SEND(ID_3_HB); // 하트비트
          CAN_SEND(ID_3_ACC); // 가속도 전송 - GET
          break;

      case CENTRAL_DOWN:
      case BROKEN_SELF:
    	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    	  HAL_CAN_Stop(&hcan);
    	  osDelay(500);// 0.5초 쉼

    	   // Recovery
    	  if (HAL_CAN_Start(&hcan) == HAL_OK) {
    		  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    		  osDelay(100);
    	  }
    	  break;
      }
	  osDelayUntil(tick); // 100Hz
	  // osDelay는 10ms+작업 주기임
  }
}

void ledTask(void *argument)
{
  SPI_Buffer_t txBuf;
  SPI_Buffer_t rxBuf; // Dummy
  uint8_t LED_COMMAND;
  volatile uint8_t SPI1_ERR_CNT = 0; // 에러 카운트
  uint8_t last_rx_seq = 0xFF; // 시퀀스 검증을 위함

  for(;;)
  {
	  	  	//  상태에 따른 명령어 결정
	        switch(state_global){
	        case OFF:         LED_COMMAND = LED_OFF; break;
	        case READY:       LED_COMMAND = LED_READY; break;
	        case DRIVE:       LED_COMMAND = LED_OFF; break;
	        case EMERGENCY:   LED_COMMAND = LED_EMERGENCY; break;
	        case MOTOR_ERR:   LED_COMMAND = LED_CENTER_DOWN; break;
	        case PEDAL_ERR:   LED_COMMAND = LED_CENTER_DOWN; break;
	        case CENTRAL_DOWN: LED_COMMAND = LED_CENTER_DOWN; break;
	        case BROKEN_SELF: LED_COMMAND = LED_BROKEN_SELF; break;
	        default:          LED_COMMAND = LED_OFF; break;
	        }

	        Build_SPI_Packet(&txBuf, LED_COMMAND); // 패킷짜기

	        // SPI 전송
	        bool frame_valid = true;
	        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	        if(HAL_SPI_TransmitReceive_DMA(&hspi1, txBuf.bytes, rxBuf.bytes, 8) == HAL_OK) {
	              // 타임아웃 20ms
	              uint32_t res = osThreadFlagsWait(FLAG_SPI_DMA_COMPLETE, osFlagsWaitAny, 20);
	              if(res == osFlagsErrorTimeout){
	                  HAL_SPI_DMAStop(&hspi1);
	                  hspi1.State = HAL_SPI_STATE_READY;
	                  frame_valid = false;
	              }
	        }
	        else {
	            frame_valid = false;
	        }
	        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	        // 데이터 검증
	        if(frame_valid){
	              if(rxBuf.pkt.header != 0xAA) frame_valid = false; // 헤더
	              if(rxBuf.pkt.tail != 0xED) frame_valid = false; // 테일
	              if(crc_xor((uint8_t *)&rxBuf.pkt, 6) != rxBuf.pkt.crc) frame_valid = false; // crc
	        }

	        // 시퀀스 확인
	        bool is_duplicate = (rxBuf.pkt.seq == last_rx_seq);

	        if(!frame_valid || is_duplicate){
	              // 에러거나 중복 데이터면 카운트 증가
	              if(++SPI1_ERR_CNT >= 5){
	                  SPI1_ERR_FLAG = 1;
	                  // 에러 상태에서도 continue 하지 않고 아래 osDelay로 넘어감
	              }
	        } else {
	              // 정상 수신
	              last_rx_seq = rxBuf.pkt.seq;
	              SPI1_ERR_CNT = 0;
	              SPI1_ERR_FLAG = 0;
	        }
	        osDelay(300);
  }
}

void accelTask(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  uint32_t interval = 10U; // 주기를 담을 변수
  for(;;)
  {
	  tick += interval;

      switch(state_global){
      case EMERGENCY:
      case DRIVE:
          Read_Accel();

          // I2C 완료 대기 (타임아웃 40ms)
          uint32_t ret = osThreadFlagsWait(FLAG_I2C_DMA_COMPLETE, osFlagsWaitAny, 40);

          if(!(ret & 0x80000000)){ // 성공 시
              Accel_ProcessData();
              taskENTER_CRITICAL(); // 48비트 짜리이므로 보호 필요. 매우 짧은 대입 연산이므로 critical section 지정하는게 나을거같음
              	  	  	  	  	  	// 단순히 레지스터에 값을 쓴까 오버헤드 거의 없고, 매우 짧은 구간이라 실시간성 손해가 거의 없을거 같아서 뮤텍스 오버헤드보다 나을거같음.
              g_accel_raw.x = (int16_t)(ax * 100.0f);
              g_accel_raw.y = (int16_t)(ay * 100.0f);
              g_accel_raw.z = (int16_t)(az * 100.0f);
              taskEXIT_CRITICAL(); // 업데이트
          }
          break;

      default:
          break;
      }
      osDelayUntil(tick);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
