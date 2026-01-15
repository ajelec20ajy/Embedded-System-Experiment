
#include "main.h"

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);

#define EMPTY 		0xAA
#define DETECTED	0xBB
#define FIRED		0xCC

typedef struct {
	uint16_t state;
}vehicle_state;

vehicle_state state_IR;
vehicle_state state_BTN;

static uint8_t btn_prev = 1; // 풀업이라 기본 1(안눌림)
static uint32_t t_last_change = 0;

uint8_t Button_PollPressed(void)
{
    uint8_t now = (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint32_t t = HAL_GetTick();

    // 상태가 바뀌었으면 시간 갱신
    if (now != btn_prev) {
        btn_prev = now;
        t_last_change = t;
        return 0; // 아직 안정화 안 됐으니 이벤트 발생 X
    }

    // 상태가 30ms 이상 유지되면 안정화로 인정
    if ((t - t_last_change) >= 30) {
        // 래치
        static uint8_t latched = 0;

        if (btn_prev == 0 && latched == 0) {
            latched = 1;   // 눌림 이벤트
            return 1;
        }
        if (btn_prev == 1) {
            latched = 0;   // 버튼 떼면 다음 눌림을 받을 준비
        }
    }

    return 0;
}

#include <stdint.h>
// IR 센서 장애물 감지되면 0나옴.
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

// CAN 관련 Zone1 주석 참고
typedef enum {
    ERR_TYPE_RX_STUCK,
    ERR_TYPE_RX_TIMEOUT,
    ERR_TYPE_TX_FAIL,
    ERR_TYPE_HW,
    ERR_TYPE_CHKSUM
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

uint8_t state_global = OFF;
uint8_t global_tx_counter = 0;

typedef struct{
	uint8_t z1;
	uint8_t z2;
	uint8_t z3;
	uint8_t z4;
	uint8_t central;
} CAN_CNT;
CAN_CNT can_rx_err_cnt;
CAN_CNT stuck_counter;
CAN_CNT last_counter;
uint32_t last_zone_rx_time;


CAN_CNT CAN_ERR_WHO ={0};
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

void CAN_ERR_HANDLE(uint32_t ID, CAN_Err_Type type){ //송,수신 포괄 에러 처리
	switch(ID){
	case ID_4_IR:
	case ID_4_BTN: // 내가 송신에 문제가 있다.
		if(type == ERR_TYPE_TX_FAIL){
			can_rx_err_cnt.z4++;
			if(can_rx_err_cnt.z4 >= 30){
				state_global = BROKEN_SELF;
				HAL_CAN_Stop(&hcan);
			}
		}
		break;
	case ID_CENTRAL_STATUS: // 마스터의 하트비트가 안들어온다
		can_rx_err_cnt.central++;
		if(can_rx_err_cnt.central>=20){
			CAN_OFF_FILTER();
			state_global = CENTRAL_DOWN;
		}
		break;

	case ID_HARDWARE_ERR: // 하드웨어에 문제가 있다.
		if(type == ERR_TYPE_HW){
			HAL_CAN_Stop(&hcan);
		}
		break;

	default:
		break;
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
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

				state_global = MOTOR_ERR;
				can_rx_err_cnt.central = 0;
				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
				break;

			case ID_CENTRAL_EMERGENCY:
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
						state_global = payload.data16.data; // 상태 반영
				}
				switch(state_global){
				case READY:
						break;
				case DRIVE:
						break;
				}
				can_rx_err_cnt.central = 0;
				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
				break;

			}
		}
	}
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
		case ID_4_IR:
			payload.data16.data = state_IR.state;
			break;
		case ID_4_BTN:
			payload.data16.data = state_BTN.state;
			break;
		case ID_4_HB:
			payload.data16.data = state_global;
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
	    	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, dummyData);
	    	  fillLevel--;
	    	  }
	      }
	      if(!any){ // 위의 경우들은 아닌경우, 포괄에러처리
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW);
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
	canFilter.FilterIdHigh = (0x000 << 5);
	canFilter.FilterIdLow = (0x000 << 5);
	canFilter.FilterMaskIdHigh = (0x000 << 5);
	canFilter.FilterMaskIdLow = (0x000 << 5);

	if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
			Error_Handler();
	}

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  state_global = OFF;
  state_IR.state = EMPTY;
  state_BTN.state = EMPTY;
  CAN_INIT();
  uint8_t ir_count = 0;
  uint32_t tick = HAL_GetTick();
  last_zone_rx_time = HAL_GetTick();
  uint8_t IR_FIRST = 0;

  while (1)
  {

	  uint32_t currentTick = HAL_GetTick();

	  if (currentTick - tick >= 10) { // Hear beat. OFF나 중앙제어기 고장 아니면 무조건 보내야됌
	          if (state_global != OFF && state_global != CENTRAL_DOWN) {
	              CAN_SEND(ID_4_HB); // 하트비트 전송
	          }
	          tick = currentTick;
	  }
	  if (state_global != OFF && state_global != CENTRAL_DOWN) {
	          if (currentTick - last_zone_rx_time > 200) { // 중앙 제어기의 하트비트가 0.2초 동안이나 안왔으면 central_down
	        	  CAN_OFF_FILTER();
	              state_global = CENTRAL_DOWN;
	          }
	  }
	  switch (state_global) {

	          case OFF:
	              // IR 센서 카운팅
	              if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
	                  if (++ir_count >= 5) { // 약 5번의 루프 동안 감지되면
	                      state_IR.state = DETECTED;
	                      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	                      if(IR_FIRST == 0){
	                      	  CAN_SEND(ID_4_IR);
	                      	  IR_FIRST = 1; // 전송했으니 플래그 잠금
	                      }
	                      // 중앙 제어기가 READY를 보내면 인터럽트에서 state_global을 변경함
	                      ir_count = 5;
	                  }
	              } else {
	                  ir_count = 0;
	                  if (IR_FIRST == 1) {
	                  	  IR_FIRST = 0;
	                  	  state_IR.state = EMPTY; // 상태도 비움으로 변경
	                  	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // LED 끄기
	                  }
	              }
	              break;

	          case READY:
	              // 버튼 폴링
	              if (Button_PollPressed() && state_BTN.state != FIRED) {
	                  state_BTN.state = FIRED;
	                	  CAN_SEND(ID_4_BTN);
	                  // 중앙 제어기가 DRIVE 혹은 EMERGENCY를 보내면 자동으로 다음 case로 이동
	              }

	              break;

	          case DRIVE:
	              // 주행 중 로직
	              state_BTN.state = EMPTY; // 다음 상황을 위해 플래그 초기화
	              break;
	          case PEDAL_ERR:
	          case MOTOR_ERR:
	          case EMERGENCY:
	        	  state_BTN.state = EMPTY;
	              break;

	          case CENTRAL_DOWN:
	        	  HAL_CAN_Stop(&hcan);
	        	  HAL_Delay(500);
	        	  if (HAL_CAN_Start(&hcan) == HAL_OK) {
	        	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	        	  HAL_Delay(100);
	        	  }
	              break;
	          case BROKEN_SELF:
	        	  HAL_CAN_Stop(&hcan);
	          default:
	              break;
	      }
	      // 1ms 주기 폴링으로 감시
	      HAL_Delay(5);
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
  hcan.Init.AutoBusOff = DISABLE;
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

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_Pin IR_Pin */
  GPIO_InitStruct.Pin = BTN_Pin|IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
