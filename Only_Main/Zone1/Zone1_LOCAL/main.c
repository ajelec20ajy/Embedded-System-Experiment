
#include "main.h"
#include "cmsis_os.h"

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;

/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SPI1 */
osThreadId_t SPI1Handle;
const osThreadAttr_t SPI1_attributes = {
  .name = "SPI1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SPI2 */
osThreadId_t SPI2Handle;
const osThreadAttr_t SPI2_attributes = {
  .name = "SPI2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for cansonicQueue */
osMessageQueueId_t cansonicQueueHandle;
const osMessageQueueAttr_t cansonicQueue_attributes = {
  .name = "cansonicQueue"
};
/* Definitions for canpedalQueue */
osMessageQueueId_t canpedalQueueHandle;
const osMessageQueueAttr_t canpedalQueue_attributes = {
  .name = "canpedalQueue"
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
void canTask(void *argument);
void spi1Task(void *argument);
void spi2Task(void *argument);

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h" // atoi
#include "string.h" // memcpy

#define FLAG_SONIC_READ            0x0001
#define FLAG_PEDAL_READ            0x0002
#define FLAG_SONIC_DMA_COMPLETE    0x0004
#define FLAG_PEDAL_DMA_COMPLETE    0x0008


#pragma pack(push,1)
typedef struct{
	uint8_t header;
	int16_t data;
	uint8_t status;
	uint8_t seq;
	uint8_t reserved;
	uint8_t crc;
	uint8_t tail;
}SPI_Packet_t;
#pragma pack(pop)

typedef union{
	SPI_Packet_t pkt;
	uint8_t bytes[8];
}SPI_Buffer_t;

int16_t g_distance;
int16_t g_accel, g_brake, g_pedal;

static SPI_Buffer_t pedal_rx;
static SPI_Buffer_t pedal_tx;
static SPI_Buffer_t sonic_rx;
static SPI_Buffer_t sonic_tx;

static uint8_t crc_xor(const uint8_t *p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= p[i];
  return c;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		osThreadFlagsSet(SPI1Handle, FLAG_SONIC_DMA_COMPLETE);
	}
	else if (hspi->Instance == SPI2) {
		osThreadFlagsSet(SPI2Handle, FLAG_PEDAL_DMA_COMPLETE);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		HAL_SPI_DMAStop(hspi);
		hspi->State = HAL_SPI_STATE_READY; // 강제로 통신 가능 상태로 복구
	}
	else if (hspi->Instance == SPI2) {
		HAL_SPI_DMAStop(hspi);
		hspi->State = HAL_SPI_STATE_READY; // 강제로 통신 가능 상태로 복구
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin){
	case DRDY_SONIC_Pin:
		osThreadFlagsSet(SPI1Handle, FLAG_SONIC_READ);
		break;
	case DRDY_PEDAL_Pin:
		osThreadFlagsSet(SPI2Handle, FLAG_PEDAL_READ);
		break;
	default:
		break;
	}
}
// CAN
#define OFF 0x01
#define READY 0x02
#define DRIVE 0x04
#define EMERGENCY 0x08
#define MOTOR_ERR 0x10
#define CENTRAL_DOWN 0x20
#define BROKEN_SELF 0x40

#define ID_1_SONIC 0x010
#define ID_1_PEDAL 0x040
#define ID_1_HB 0x071

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


// RTOS용 FLAG
#define FLAG_STATE_OFF			0x11
#define FLAG_STATE_READY		0x22
#define FLAG_STATE_DRIVE		0x44
#define FLAG_STATE_EMERGENCY	0x88
#define FLAG_STATE_MOTOR_ERR	0xAA

typedef enum { // 에러 종류
    ERR_TYPE_RX_STUCK,    // 데이터 갱신 안됨
    ERR_TYPE_RX_TIMEOUT,  // 메시지 자체가 안옴 (heartbeat)
    ERR_TYPE_TX_FAIL,     // 송신 실패 (메일박스 꽉참 )
    ERR_TYPE_HW,          // 하드웨어 에러 (bus-off 등)
    ERR_TYPE_CHKSUM       // 데이터 변조 감지
} CAN_Err_Type;

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
    uint8_t bytes[8];
} CAN_Payload_t;

typedef struct{
	uint8_t z1;
	uint8_t z2;
	uint8_t z3;
	uint8_t z4;
	uint8_t central;
} CAN_CNT;

CAN_CNT stuck_counter;
CAN_CNT last_counter;
CAN_CNT can_rx_err_cnt;
uint32_t last_zone_rx_time;
uint8_t state_global = OFF;
uint8_t global_tx_counter = 0;
volatile uint8_t FLAG_CAN_STOP = 0;

void Put_Latest_Data(osMessageQueueId_t queueHandle, int16_t data){ // Size 1 Queue Overwrite하기
	int16_t msg = data;
	    if(osMessageQueueGetCount(queueHandle) > 0){
	        int16_t dummy;
	        osMessageQueueGet(queueHandle, &dummy, NULL, 0);
	    }
	    osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void CAN_ERR_HANDLE(uint32_t ID, CAN_Err_Type type){
	switch(ID){
	case ID_1_SONIC:
	case ID_1_PEDAL:
		can_rx_err_cnt.z1++;
		if(can_rx_err_cnt.z1>=20){
			FLAG_CAN_STOP = 1;
			HAL_CAN_Stop(&hcan);
			state_global = BROKEN_SELF;
		}
		break;
	case ID_CENTRAL_STATUS:
		can_rx_err_cnt.central++;
		if(can_rx_err_cnt.central>=20 || type == ERR_TYPE_RX_TIMEOUT){
			state_global = CENTRAL_DOWN;
		}
		break;

	case ID_HARDWARE_ERR:
		if(type == ERR_TYPE_HW){
			FLAG_CAN_STOP = 1;
			HAL_CAN_Stop(&hcan);
			state_global = BROKEN_SELF;
		}
		break;

	default:
		break;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	CAN_Payload_t payload = { 0 };
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, payload.bytes) == HAL_OK){
    	if(rxHeader.IDE == CAN_ID_STD){
    			uint8_t received_sum = 0;
    			for (int i = 0; i < 7; i++) { // Checksum
    				received_sum += payload.bytes[i];
    			}
    			if (received_sum != payload.bytes[7]) {
    				CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_CHKSUM);
    				return; // 이번거 데이터 오염되었다 판단, 그냥 건너뜀
    			}
    			switch (rxHeader.StdId) {
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

    				state_global = MOTOR_ERR;
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
    				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
    				break;
    			case ID_CENTRAL_STATUS:
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

    				if (rxHeader.DLC == 8) {
    					state_global = payload.data16.data;
    				}
    				can_rx_err_cnt.central = 0;
    				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
    				break;
    			}
    	}
    }
}

void CAN_INIT() {
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

	canFilter.FilterBank = 1;
	canFilter.FilterIdHigh = (0x001 << 5);
	canFilter.FilterIdLow = (0x000 << 5);
	canFilter.FilterMaskIdHigh = (0x000 << 5);
	canFilter.FilterMaskIdLow = (0x000 << 5);

	if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
		Error_Handler();
	}

	// CAN 설정 2.
	last_zone_rx_time = HAL_GetTick();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_SEND(uint32_t ID) {
	    CAN_TxHeaderTypeDef txHeader ={0};
	    uint32_t txMailbox;
	    CAN_Payload_t payload ={0};


	    txHeader.IDE = CAN_ID_STD;
	    txHeader.RTR = CAN_RTR_DATA;
	    txHeader.TransmitGlobalTime = DISABLE;
	    txHeader.StdId = ID;
	    txHeader.DLC = 8;

	    switch(ID){
		case ID_1_SONIC:
			payload.data16.data = g_distance;
			break;
		case ID_1_PEDAL:
			if(state_global == DRIVE || state_global == CENTRAL_DOWN) payload.data16.data = g_pedal;
			else payload.data16.data = 0;
			break;
		case ID_1_HB: // heartbeat
			payload.data16.data = state_global;
			break;
		default:
			break;
	    }

	    payload.data16.counter = global_tx_counter++;
	    uint8_t sum = 0;
	        for (int i = 0; i < 7; i++) {
	        	sum += payload.bytes[i];
	        }
	        payload.data16.checksum = sum;

	    uint32_t tick = HAL_GetTick();
	    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
	    	if (HAL_GetTick() - tick > 10) {
	    		CAN_ERR_HANDLE(ID, ERR_TYPE_TX_FAIL); // 메일박스 고임 타임아웃처리
	    		return; // 타임아웃 시 포기
	    	}
	    }

	    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, payload.bytes, &txMailbox) != HAL_OK) {
	    	CAN_ERR_HANDLE(ID, ERR_TYPE_HW); // 창구에서 거절
	    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
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
	    	  // 메시지를 읽는 것 자체가 FIFO를 비우는 동작입니다.
	    	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, dummyData);
	    	  fillLevel--;
	    	  }
	      }
	      if(!any){ // 위의 경우들은 아닌경우, 포괄에러처리
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW);
	      }
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();

  HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_PEDAL_GPIO_Port, CS_PEDAL_Pin, GPIO_PIN_SET);
  CAN_INIT();

  osKernelInitialize();

  cansonicQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &cansonicQueue_attributes);
  canpedalQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &canpedalQueue_attributes);

  CANHandle = osThreadNew(canTask, NULL, &CAN_attributes);
  SPI1Handle = osThreadNew(spi1Task, NULL, &SPI1_attributes);
  SPI2Handle = osThreadNew(spi2Task, NULL, &SPI2_attributes);

  osKernelStart();

  while (1)
  {

  }
}

void canTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint16_t rx_msg;
	  uint32_t tick = osKernelGetTickCount();
	  const uint32_t kPeriod = 10U; // 10ms 주기 (100Hz)
	  uint8_t state_local = OFF;
  /* Infinite loop */
  for(;;)
  {
	  tick += kPeriod;
	  state_local = state_global;
	  if ((state_local != OFF) && (state_local != CENTRAL_DOWN) && (state_local != BROKEN_SELF))
	        {
	            if (HAL_GetTick() - last_zone_rx_time > 200) {
	          	  state_local = CENTRAL_DOWN;
	            }
	        }
	  switch(state_local){
	        case DRIVE:
                if (osMessageQueueGet(canpedalQueueHandle, &rx_msg, NULL, 0) == osOK) {
                    // rx_msg 값을 state_PEDAL.state 등에 반영하는 로직 필요 시 추가
                    CAN_SEND(ID_1_PEDAL);
                }

                // 초음파 데이터 전송 (Queue에 데이터가 있을 때만)
                if (osMessageQueueGet(cansonicQueueHandle, &rx_msg, NULL, 0) == osOK) {
                    CAN_SEND(ID_1_SONIC);
                }
	        case READY:
	        case EMERGENCY:
	        case MOTOR_ERR:
	        	CAN_SEND(ID_1_HB);
	        	break;
	        case OFF:
	      	  break;
	        case CENTRAL_DOWN:
	        	CAN_SEND(ID_1_PEDAL);
	          break;
	        case BROKEN_SELF:
	      	  HAL_CAN_Stop(&hcan);
	      	  break;
	  }
	  osDelayUntil(tick); // 100Hz
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_spi1Task */
/**
* @brief Function implementing the SPI1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spi1Task */
void spi1Task(void *argument)
{
  /* USER CODE BEGIN spi1Task */
  /* Infinite loop */
  for(;;)
  {
		// 페달 DRDY 인터럽트에서 플래그 세워주길 대기
	    uint32_t result = osThreadFlagsWait(FLAG_SONIC_READ, osFlagsWaitAny, osWaitForever);

	    if (result & FLAG_SONIC_READ) { // DRDY ISR에서 플래그처리함.
	    	// 초음파 통신 수행 (CS_SONIC)
	    	HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_RESET);
	    	if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&sonic_tx, (uint8_t*)&sonic_rx, 8) == HAL_OK){
	    		uint32_t sonic_res = osThreadFlagsWait(FLAG_SONIC_DMA_COMPLETE, osFlagsWaitAny, 20); // 완료 대기
	    		if(sonic_res & osFlagsErrorTimeout){
	    			HAL_SPI_DMAStop(&hspi1);
	    			hspi1.State = HAL_SPI_STATE_READY;
	    		}
	    	}
	    	HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_SET);
	    	g_distance = sonic_rx.pkt.data; // 초음파 측정값 업데이트
	    	Put_Latest_Data(cansonicQueueHandle, g_distance);
	    }
	    osDelay(1);
  }
  /* USER CODE END spi1Task */
}

/* USER CODE BEGIN Header_spi2Task */
/**
* @brief Function implementing the SPI2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spi2Task */
void spi2Task(void *argument)
{
  /* USER CODE BEGIN spi2Task */
  /* Infinite loop */
  for(;;)
  {
	  // 페달 DRDY 인터럽트에서 플래그 세워주길 대기
	  uint32_t result = osThreadFlagsWait(FLAG_PEDAL_READ, osFlagsWaitAny, osWaitForever);
	  if (result & FLAG_PEDAL_READ) { // DRDY ISR에서 플래그처리함.
		  // 초음파 통신 수행 (CS_SONIC)
	 	  HAL_GPIO_WritePin(CS_PEDAL_GPIO_Port, CS_PEDAL_Pin, GPIO_PIN_RESET);
	 	  if(HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)&pedal_tx, (uint8_t*)&pedal_rx, 16) == HAL_OK){
	 		  uint32_t pedal_res = osThreadFlagsWait(FLAG_PEDAL_DMA_COMPLETE, osFlagsWaitAny, 20); // 완료 대기
	 		  	  if(pedal_res & osFlagsErrorTimeout){
	 		  		  HAL_SPI_DMAStop(&hspi2);
	 		  		  hspi2.State = HAL_SPI_STATE_READY;
	 		  	  }
	 	  } // 8바이트로 단일 패킷 받아도 되는데.. 뭔가 분석하려고 악셀/브레이크 나눠서 받음. 근데 할게없네
	 	  HAL_GPIO_WritePin(CS_PEDAL_GPIO_Port, CS_PEDAL_Pin, GPIO_PIN_SET);
	 	  g_pedal = pedal_rx.pkt.data;
	 	  Put_Latest_Data(canpedalQueueHandle, g_pedal);
	  }
	  osDelay(1);
  }
  /* USER CODE END spi2Task */
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_PEDAL_GPIO_Port, CS_PEDAL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_SONIC_Pin DRDY_PEDAL_Pin */
  GPIO_InitStruct.Pin = DRDY_SONIC_Pin|DRDY_PEDAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SONIC_Pin */
  GPIO_InitStruct.Pin = CS_SONIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SONIC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_PEDAL_Pin */
  GPIO_InitStruct.Pin = CS_PEDAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_PEDAL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}



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
