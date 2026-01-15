//letgo
#include "main.h"

ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

#include <stdio.h>
#include <string.h>

#define SPI_PKT_LEN 8

#pragma pack(push, 1)
typedef struct {
    uint8_t header;    // 0XAA ** SPI에서 헤더 테일 전부 통일
    uint16_t data;    // 데이터
    uint8_t status;    // 상태
    uint8_t seq;       // 시퀀스 번호
    uint8_t reserved;  // 사이즈맞출라고 리저브드
    uint8_t crc;       // CRC (0~5번 바이트 XOR)
    uint8_t tail;      // 0xED
} SPI_Packet_t;
#pragma pack(pop)

typedef union {
    SPI_Packet_t pkt;
    uint8_t bytes[SPI_PKT_LEN];
} SPI_Buffer_t; //바이트로도 접근하기 편하게 유니온 처리

static SPI_Buffer_t tx_buf, rx_buf; // SPI 통신용 버퍼
static uint8_t g_seq = 0;


volatile uint8_t g_new_data_ready = 0; // 새로운 거리 데이터 측정 완료 플래그
uint32_t last_trig_tick = 0;           // 마지막 트리거 시간
volatile uint8_t g_spi_xfer_done = 1; // 1: 전송 완료됨(IDLE), 0: 전송 중(BUSY)

static uint8_t crc_xor(const uint8_t *p, int n){ // CRC 바이트 만들어주는 함수
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= p[i];
  return c;
}

uint16_t Get_ADC_Value(uint32_t channel){ // ADC 읽어오기
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

	// 채널 설정 적용
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10); // 변환 완료 대기
	uint16_t val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return val;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if (hspi->Instance != SPI1) return;

  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);
  g_spi_xfer_done = 1; // 플래그 설정 (메인 루프 잠금 해제)
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
  if (hspi->Instance != SPI1) return;
  g_spi_xfer_done = 1; // 다음 루프 돌 수 있게 해제
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  HAL_Delay(1000);
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);
    g_spi_xfer_done = 1;
  while (1)
  {
	  // 이전 전송이 끝났을 때만 새로운 데이터를 준비함 (시퀀스 보호)
	        if (g_spi_xfer_done == 1)
	        {
	            // 1. ADC 값 읽기
	            int16_t adc_accel = Get_ADC_Value(ADC_CHANNEL_0);
	            int16_t adc_brake = Get_ADC_Value(ADC_CHANNEL_1);
	            int16_t pedal = adc_accel - adc_brake;

	            if(pedal < 0) pedal = 0;
	            if(pedal > 3000) pedal = 3000;

	            // 2. TX 패킷 조립
	            tx_buf.pkt.header = 0xAA;
	            tx_buf.pkt.data = pedal;
	            tx_buf.pkt.status = 0;
	            tx_buf.pkt.seq = g_seq++; // 여기서 시퀀스 증가
	            //tx_buf.pkt.seq = 0; // PEDAL_ERR 유도
	            tx_buf.pkt.reserved = 0x00;
	            tx_buf.pkt.crc = crc_xor((uint8_t*)&tx_buf.pkt, 6);
	            tx_buf.pkt.tail = 0xED;

	            // 3. SPI DMA 시작 -마스터가 클럭 줄 때까지 대기
	            // 마스터가 CS 내리기 전에 미리 호출되어 있어야 함
	            g_spi_xfer_done = 0; // BUSY 상태로 변경
	            if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_buf, (uint8_t*)&rx_buf, SPI_PKT_LEN) != HAL_OK)
	            {
	                // DMA 설정 실패 시 복구
	                g_spi_xfer_done = 1;
	                HAL_Delay(10);
	                continue;
	            }

	            // 4. 마스터에게 읽으라고 알리기
	            HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_SET);

	            // 5. 타임아웃 체크를 위한 틱 저장
	            last_trig_tick = HAL_GetTick();
	        }
	        else
	        {
	            // 전송 중일 때 (마스터가 아직 안 읽어감)
	            // 마스터가 너무 오랫동안 안 읽어가면 강제 리셋
	            if (HAL_GetTick() - last_trig_tick > 50)
	            {
	                HAL_SPI_DMAStop(&hspi1);
	                HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET); // 신호 철회
	                hspi1.State = HAL_SPI_STATE_READY;
	                g_spi_xfer_done = 1; // 다시 루프 돌도록 허용
	            }
	        }

	        HAL_Delay(1); // 루프 속도 조절
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRDY_GPIO_Port, DRDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

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
