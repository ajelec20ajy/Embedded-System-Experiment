// letgo
#include "main.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);

#include "stdlib.h"

#define LED_OFF		0xFF // LED 모두 꺼라
#define LED_READY		0x01 // 환영등(Zone 4의 IR 감지 시): LED1 ON, LED2 OFF
#define LED_EMERGENCY 	0x02 // 비상등(Zone 1, 2의 충돌 감지 시): LED1 OFF, LED2 ON
#define LED_CENTER_DOWN	0x04 // 두 LED 모두 깜빡여서 중앙 Down 알림
#define LED_BROKEN_SELF 0x08 // 파란 LED 깜빡여서 스스로의 CAN 고장을 알림

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

SPI_Buffer_t rx_buf; // 마스터로부터 받은 데이터
SPI_Buffer_t tx_buf; // 마스터에게 보낼 데이터 (상태 정보 등)

volatile uint8_t spi_rx_flag = 0; // SPI 들어왔는지 나타내는 플래그
uint32_t last_comm_tick = 0; // 시간 감시

uint8_t last_rx_cnt = 0; // 수신 카운터
uint8_t tx_cnt = 0; // 송신 카운터

static uint8_t crc_xor(const uint8_t *p, int n){ // CRC만드는 함수
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= p[i];
  return c;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) { // SPI 콜백
	if (hspi->Instance == SPI1) {
	        spi_rx_flag = 1; // 마스터에게 잘 받았어요
	        last_comm_tick = HAL_GetTick(); // 통신 성공 시간 기록

	        tx_buf.pkt.seq = tx_cnt++;
	        tx_buf.pkt.crc = crc_xor((uint8_t*)&tx_buf.pkt, 6);
	        HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf.bytes, rx_buf.bytes, 8); // 바로 다음 SPI DMA 수신 준비함
	 }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
	        spi_rx_flag = 2;
	        last_comm_tick = HAL_GetTick(); // 통신 성공 시간 기록
	        HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf.bytes, rx_buf.bytes, 8);
	 }
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();

  uint8_t LED_COMMAND = LED_OFF;
  tx_buf.pkt.header = 0xAA;
  tx_buf.pkt.data = 0;
  tx_buf.pkt.status = 0;
  tx_buf.pkt.reserved = 0;
  tx_buf.pkt.seq = tx_cnt++;
  tx_buf.pkt.crc = crc_xor((uint8_t*)&tx_buf.pkt, 6);
  tx_buf.pkt.tail = 0xED;
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf.bytes, rx_buf.bytes, 8);
  uint8_t cnt = 0;

  while (1)
  {
	  if(spi_rx_flag == 1){ // 마스터에게 잘 받은 경우
		  spi_rx_flag = 0;
		  if(rx_buf.pkt.header == 0xAA && rx_buf.pkt.tail == 0xED){ // 헤더와 테일 검증
			  if(crc_xor(rx_buf.bytes, 6) == rx_buf.pkt.crc){ // CRC 검증
				  if(rx_buf.pkt.seq == last_rx_cnt) continue; // 시퀀스 검증
				  else last_rx_cnt = rx_buf.pkt.seq;
				  LED_COMMAND = rx_buf.pkt.data; // 데이터 검증 후 LED 명령 업데이트
			  }

		  }
	  }
	  switch(LED_COMMAND){ // LED 명령에 따라 업데이트
	  case LED_OFF: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  break; // LED 다 끔

	  case LED_READY: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  break; // 파랑불은 키고 빨강불은 끔

	  case LED_EMERGENCY: cnt++; HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); if(cnt >= 10) {HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); cnt = 0;}
		  break; // 파랑불은 끄고 0.5주기로 빨강불 토글

	  case LED_CENTER_DOWN: cnt++; if(cnt >= 10){HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); cnt = 0;}
		  break; //둘 다 토글

	  case LED_BROKEN_SELF: cnt++; if(cnt >= 10){HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); cnt = 0;} HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		  break; // 파랑불은 토글, 빨강불은 킴

	  default: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  break; // 기본, 둘 다 끔
	  }
	  HAL_Delay(50); // 폴링으로 spi dma 수신 완료한거 있는지 플래그를 체크

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
