
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;

/* Definitions for SPI1 */
osThreadId_t SPI1Handle;
const osThreadAttr_t SPI1_attributes = {
  .name = "SPI1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SPI2 */
osThreadId_t SPI2Handle;
const osThreadAttr_t SPI2_attributes = {
  .name = "SPI2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
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
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
void spi1Task(void *argument);
void canTask(void *argument);
void spi2Task(void *argument);

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h" // atoi

#include "string.h" // memcpy
// 자고로 FLAG는 한 비트만 1이게 구성하는게 &로 처리하기 편해요.
#define FLAG_SONIC_READ            0x0001 // SPI1으로 초음파 데이터 읽어가세요 플래그
#define FLAG_MOTOR_SEND            0x0002 // SPI2로 모터 드라이버 MCU한테 전송하세요 플래그
#define FLAG_SONIC_DMA_COMPLETE    0x0004 // SPI1 DMA 수신 완료
#define FLAG_MOTOR_DMA_COMPLETE    0x0008 // SPI2 DMA 전송 완료


#pragma pack(push,1) // 이렇게하면 패딩 안생긴다네요 그냥 참고.
typedef struct{
	uint8_t header;
	int16_t data; // dataH<<4 | dataL
	uint8_t status;
	uint8_t seq;
	uint8_t reserved;
	uint8_t crc;
	uint8_t tail;
}SPI_Packet_t; // SPI 데이터 패킷
#pragma pack(pop)

typedef union{
	SPI_Packet_t pkt;
	uint8_t bytes[8];
}SPI_Buffer_t;

int16_t g_distance;
int16_t g_pedal=0; // Zone1한테 받은거
int16_t g_pedal_from_driver; // Zone2의 모터 드라이버 MCU에게 보고받은거

// SPI용 버퍼. VOLATILE! 매번 체크
volatile SPI_Buffer_t pedal_rx;
volatile SPI_Buffer_t pedal_tx;
volatile SPI_Buffer_t sonic_rx;
volatile SPI_Buffer_t sonic_tx;

volatile uint8_t SPI1_ERR_FLAG = 0; //SPI1의 에러를 알려줄 플래그
volatile uint8_t SPI2_ERR_FLAG = 0; //SPI2의 에러를 알려줄 플래그


static uint8_t crc_xor(const uint8_t *p, int n) { // CRC 계산해주는 함수
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= p[i];
  return c;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) { // 초음파 수신 완료
		osThreadFlagsSet(SPI1Handle, FLAG_SONIC_DMA_COMPLETE); // spi1으로 온 콜백이면 spi1Task 깨우기(초음파)
	}
	else if (hspi->Instance == SPI2) { // 모터 드라이버 전송 완료
		osThreadFlagsSet(SPI2Handle, FLAG_MOTOR_DMA_COMPLETE); // spi2로 온 콜백이면 spi2Task 깨우기(모터드라이버)
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		HAL_SPI_DMAStop(hspi); // 멈추고
		hspi->State = HAL_SPI_STATE_READY; // 강제로 통신 가능 상태로 복구
	}
	else if (hspi->Instance == SPI2) {
		HAL_SPI_DMAStop(hspi);
		hspi->State = HAL_SPI_STATE_READY;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == DRDY_Pin){
		osThreadFlagsSet(SPI1Handle, FLAG_SONIC_READ); // 초음파 데이터 읽어가세용 spi1Task 꺠움.
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

// 구조체들
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
} CAN_Payload_t; // 데이터 내부 패킷

typedef struct{
	uint8_t z1;
	uint8_t z2;
	uint8_t z3;
	uint8_t z4;
	uint8_t central;
} CAN_CNT; // 여러 카운팅을 위함

typedef struct {
    uint32_t id; // 4 bytes
    uint8_t data[8]; // 8 bytes
} CAN_Msg_t;

CAN_CNT stuck_counter; // 어디서 에러가 났는지 구분해서 카운팅하는 변수
CAN_CNT last_counter; // 패킷의 카운터를 통해 스턱 여부를 결정하기 위함
CAN_CNT can_rx_err_cnt; // 에러 카운터
uint32_t last_zone_rx_time; // 하트비트 감시용 타임 기록
volatile uint8_t state_global = OFF; // 항상 직접 메모리에서 접근하게.
uint8_t global_tx_counter = 0; // CAN 송신 패킷에 넣을 카운터

void Put_Latest_Data(osMessageQueueId_t queueHandle, int16_t data){ // Size 1 Queue Overwrite하기
	int16_t msg = data;
	    if(osMessageQueueGetCount(queueHandle) > 0){ // 1개 이상이면
	        int16_t dummy;
	        osMessageQueueGet(queueHandle, &dummy, NULL, 0); // 더미에다 빼버리고
	    }
	    osMessageQueuePut(queueHandle, &msg, 0, 0); // 새거 넣음
}

void CAN_OFF_FILTER(){ // CAN 필터 꺼버리기
	CAN_FilterTypeDef canFilter;
	// Zone에서 Hearbeat이제 안보내고 안받음. 이제 CAN은 ZONE1<->ZONE2만 씀.
	canFilter.FilterBank = 0;                       // 필터 뱅크 0번 사용
	canFilter.FilterMode = CAN_FILTERMODE_IDLIST;   // 리스트 모드
	canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter.FilterIdHigh = (0x001 << 5); // 모터 고장 ID
	canFilter.FilterIdLow = (0x002 << 5); // EMERGENCY 전용
	canFilter.FilterMaskIdHigh = (0x004 << 5); // 중앙 제어기의 상태 전파
	canFilter.FilterMaskIdLow = (0x000 << 5); // Zone1의 페달 데이터
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;  // FIFO 0번에 할당
	canFilter.FilterActivation = DISABLE;            // 필터 활성화
	canFilter.SlaveStartFilterBank = 14;
	canFilter.FilterActivation = CAN_FILTER_DISABLE;
	HAL_CAN_ConfigFilter(&hcan, &canFilter);
}

void CAN_ERR_HANDLE(uint32_t ID, CAN_Err_Type type){ // CAN 송,수신 포괄 에러 처리
	switch(ID){
	case ID_2_SONIC:
	case ID_2_MOTOR_STATUS:
	case ID_2_MOTOR_SPI_ERR:
		can_rx_err_cnt.z2++;
		if(can_rx_err_cnt.z2>=20){
			state_global = BROKEN_SELF;
			osThreadFlagsSet(SPI2Handle, FLAG_MOTOR_SEND); // 깨우고
			// 주체가 나임. 내 can이 망가진거면 can 끄고 비상정지 해야됌. 왜냐면 어짜피 페달 데이터나 Emergency 상태 전파를 못받으니깐
			HAL_CAN_Stop(&hcan);
		}
		break;
	case ID_CENTRAL_STATUS:
		can_rx_err_cnt.central++;
		if(can_rx_err_cnt.central>=20 || type == ERR_TYPE_RX_TIMEOUT){
			CAN_OFF_FILTER();
				state_global = CENTRAL_DOWN;  // 주체가 너임 -> CAN STOP 안함. 왜냐? Zone1으로부터 Pedal 정보를 게속 받기 위해서.
		}
		break;

	case ID_HARDWARE_ERR:
		if(type == ERR_TYPE_HW){
			state_global = BROKEN_SELF;
			osThreadFlagsSet(SPI2Handle, FLAG_MOTOR_SEND); // 모터 깨워서 0 보내게 함
			HAL_CAN_Stop(&hcan); // 내가 망가졌으면 그냥 꺼버리고 Isolation시킴
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

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, payload.bytes) == HAL_OK){
    	if(rxHeader.IDE == CAN_ID_STD){
    			uint8_t received_sum = 0;
    			for (int i = 0; i < 7; i++) { // Checksum
    				received_sum += payload.bytes[i];
    			}
    			if (received_sum != payload.bytes[7]) { // payload.byte[7] = 맨끝은 체크섬임
    				CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_CHKSUM);
    				return; // 이번거 데이터 오염되었다 판단, 그냥 건너뜀
    			}
    			switch (rxHeader.StdId) {
    			case ID_CENTRAL_MOTOR_ERR:
    				if (payload.data16.counter == last_counter.central) { // 카운터(시퀀스) 검사하기
    					if (++stuck_counter.central > 5) { // 5번이상 카운터가 어긋남(패킷 변조)
    						CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_RX_STUCK); // 핸들러 실행
    						return; // 처리 후 return
    					}
    				}
    				else {
    					stuck_counter.central = 0;
    					last_counter.central = payload.data16.counter;
    				}
    				last_zone_rx_time = HAL_GetTick(); // 어잿든 중앙제어기한테 온 메시지니 RX 타임 갱신
    				state_global = MOTOR_ERR; // 상태반영
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
    				last_zone_rx_time = HAL_GetTick();
    				state_global = EMERGENCY;
    				g_pedal = 0;
    				osThreadFlagsSet(SPI2Handle, FLAG_MOTOR_SEND);
    				break;
    			case ID_CENTRAL_STATUS:
    				if (payload.data16.counter == last_counter.central) { // 어라라 저번에 온거랑 카운터(스퀀스)가 똑같네? 그럼 중앙 제어기 이상한데
    					if (++stuck_counter.central > 5) { // 5번넘게 이상하면
    						CAN_ERR_HANDLE(rxHeader.StdId, ERR_TYPE_RX_STUCK); // 에러핸들러 실행
    						return; // 처리 후 return
    					}
    				}
    				else { // 음 카운터가 잘 증가되어서 왔군
    					stuck_counter.central = 0;
    					last_counter.central = payload.data16.counter;
    				}

    				if (rxHeader.DLC == 8) {
    					state_global = payload.data16.data; // 상태 업데이트 (atomic)

    				}
    				can_rx_err_cnt.central = 0;
    				last_zone_rx_time = HAL_GetTick(); // 중앙제어기 HeartBeat 감지용
    				break;
    			case ID_1_PEDAL: // DRIVE 상태에서 전송됨이 송신측인 Zone1측에서 보장됌. CENTRAL_DOWN시에도 이 ID로 ZONE1으로부터 페달값 계속 수신
        			g_pedal = payload.data16.data;
        			osThreadFlagsSet(SPI2Handle, FLAG_MOTOR_SEND);
        			break;
    			}
    	}
    }
}

void CAN_SEND(uint32_t ID) {
	    CAN_TxHeaderTypeDef txHeader ={0};
	    uint32_t txMailbox;
	    CAN_Payload_t payload ={0};

	    // CAN 필드 설정
	    txHeader.IDE = CAN_ID_STD; // 표준 ID
	    txHeader.RTR = CAN_RTR_DATA; // 데이터 프레임임
	    txHeader.TransmitGlobalTime = DISABLE;
	    txHeader.StdId = ID;
	    txHeader.DLC = 8; // 데이터 8바이트에요
	    payload.data16.counter = global_tx_counter++;
	    switch(ID){ // ID에 따라 실을 데이터 결정
		case ID_2_SONIC:
			payload.data16.data = g_distance; //초음파거리
			break;
		case ID_2_MOTOR_STATUS:
			payload.data16.data = g_pedal_from_driver; //페달측정값
			//payload.data16.counter = 0; // 의도적인 고장 EMERGENCY
			break;
		case ID_2_MOTOR_SPI_ERR:
			payload.data16.data = 0;
			break;
		case ID_2_HB:
			payload.data16.data = state_global; //HEARTBEAT론 현 반영 상태 보냄
			//payload.data16.counter = 0; // 의도적인 고장 EMERGENCY
	    }

	    uint8_t sum = 0;
	    for (int i = 0; i < 7; i++) {
	    	sum += payload.bytes[i];
	    }
	    payload.data16.checksum = sum; // Checksum 삽입

	    uint32_t tick = HAL_GetTick();
	    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
	    	if (HAL_GetTick() - tick > 10) {
	    		CAN_ERR_HANDLE(ID, ERR_TYPE_TX_FAIL); // 메일박스 고임 타임아웃처리
	    		return; // 타임아웃 시 포기
	    	}
	    }

	    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, payload.bytes, &txMailbox) != HAL_OK) {
	    	CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW); // 창구에서 거절
	    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	      uint32_t err = HAL_CAN_GetError(hcan);
	      uint8_t any = 0;

	      if(err & HAL_CAN_ERROR_BOF){
	    	  any = 1;
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW); // 하드웨어적 에러
	      }

	      if(err & HAL_CAN_ERROR_ACK){ // ACK 응답이 없는 경우
	    	  any = 1;
	    	  can_rx_err_cnt.central++; // 중앙 제어기 이상하다 +1
	      }

	      if(err & HAL_CAN_ERROR_RX_FOV0){ // FIFO가 가득 찼을때
	    	  any = 1;
	    	  CAN_RxHeaderTypeDef rxHeader;
	    	  uint8_t dummyData[8];

	    	  // FIFO 0의 메시지 개수를 확인하여 모두 읽어서 버림
	    	  uint32_t fillLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

	    	  while (fillLevel > 0) {
	    	  // 메시지 읽으면 FIFO 비워짐
	    	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, dummyData);
	    	  fillLevel--;
	    	  }
	      }
	      if(!any){ // 위의 경우들은 아닌경우, 포괄에러처리
	    	  CAN_ERR_HANDLE(ID_HARDWARE_ERR, ERR_TYPE_HW);
	      }
}

void CAN_INIT(){
	CAN_FilterTypeDef canFilter;

	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter.FilterIdHigh = (0x001 << 5); // 모터 고장 ID
	canFilter.FilterIdLow = (0x002 << 5); // EMERGENCY 전용
	canFilter.FilterMaskIdHigh = (0x004 << 5); // 중앙 제어기의 상태 전파
	canFilter.FilterMaskIdLow = (0x000 << 5); // Zone1의 페달 데이터
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;  // FIFO 0번에 할당
	canFilter.FilterActivation = ENABLE;            // 필터 활성화
	canFilter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
	    Error_Handler();
	}

	canFilter.FilterBank = 1;
	canFilter.FilterIdHigh = (0x040 << 5);
	canFilter.FilterIdLow = (0x000 << 5);
	canFilter.FilterMaskIdHigh = (0x000 << 5);
	canFilter.FilterMaskIdLow = (0x000 << 5);

	if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
		Error_Handler();
	}
	// CAN 설정
	last_zone_rx_time = HAL_GetTick(); // last_zone_rx_time : Hearbeat용
	HAL_CAN_Start(&hcan); // 시작
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 콜백수신 시작

}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();

  CAN_INIT();

  osKernelInitialize();

  cansonicQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &cansonicQueue_attributes); // 큐 설정
  canpedalQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &canpedalQueue_attributes);

  SPI1Handle = osThreadNew(spi1Task, NULL, &SPI1_attributes); // Task 설정
  CANHandle = osThreadNew(canTask, NULL, &CAN_attributes);
  SPI2Handle = osThreadNew(spi2Task, NULL, &SPI2_attributes);

  osKernelStart(); // 태스크 스케줄링 시작

  while (1)
  {
  }
}

void spi1Task(void *argument)
{
	  uint8_t SPI1_ERR_CNT = 0;
	  uint8_t last_rx_seq = 0xFF;
	  uint8_t seq = 0;

	  for(;;)
	  {
	    // [1] DRDY 신호 대기
	    uint32_t result = osThreadFlagsWait(FLAG_SONIC_READ, osFlagsWaitAny, 150);

	    // [2] 초음파 값 받기
	    if (result & FLAG_SONIC_READ) {

	        // 전송할 TX 패킷 만들기(더미) 필요없는데 한번 해봄
	        sonic_tx.pkt.header = 0xAA;
	        sonic_tx.pkt.data = 0;
	        sonic_tx.pkt.status = state_global;
	        sonic_tx.pkt.seq = seq++;
	        sonic_tx.pkt.reserved = 0;
	        sonic_tx.pkt.crc = crc_xor(sonic_tx.bytes, 6);
	        sonic_tx.pkt.tail = 0xED;

	    	bool frame_valid = true;

	    	HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_RESET); // CS Low
	    	if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&sonic_tx, (uint8_t*)&sonic_rx, 8) == HAL_OK){
	    		uint32_t sonic_res = osThreadFlagsWait(FLAG_SONIC_DMA_COMPLETE, osFlagsWaitAny, 20);

	    		if(sonic_res == osFlagsErrorTimeout){ // DMA 타임아웃
	    			HAL_SPI_DMAStop(&hspi1);
	    			hspi1.State = HAL_SPI_STATE_READY;
	    			frame_valid = false;
	    		}
	    	}
	    	else{
	    		frame_valid = false; // 불량 처리
	    	}
	    	HAL_GPIO_WritePin(CS_SONIC_GPIO_Port, CS_SONIC_Pin, GPIO_PIN_SET); // CS High

	    	// 데이터 검증
	    	if(frame_valid){
	        	if(sonic_rx.pkt.header != 0xAA) frame_valid = false;
	        	if(sonic_rx.pkt.tail != 0xED) frame_valid = false;
	        	if(crc_xor((uint8_t *)&sonic_rx.pkt, 6) != sonic_rx.pkt.crc) frame_valid = false;
	    	}

	        bool skip_update = false;

	    	if(!frame_valid){
	    		if(++SPI1_ERR_CNT >= 5){

	    			SPI1_ERR_FLAG = 1;
	    		}
	    		skip_update = true; // 이번 턴은 업데이트 건너뜀
	    	}
	        else if (sonic_rx.pkt.seq == last_rx_seq) {
	    	    if (++SPI1_ERR_CNT >= 5)
	    	        SPI1_ERR_FLAG = 1;
	    	    skip_update = true; // 중복 데이터니 업데이트 건너뜀
	    	}
	        else {
	            // 정상 데이터 수신
	        	last_rx_seq = sonic_rx.pkt.seq;
	        	SPI1_ERR_CNT = 0;
	        	SPI1_ERR_FLAG = 0;
	            skip_update = false;
	        }

	        //  유효할 때만 값 업데이트 및 큐 전송
	        if(!skip_update) {
	        	g_distance = sonic_rx.pkt.data; // 값 업데이트
	        	Put_Latest_Data(cansonicQueueHandle, g_distance); // 큐로 보냄
	        }
	    }
	  }
}

void canTask(void *argument)
{
	uint16_t rx_msg;
	uint32_t tick = osKernelGetTickCount();
	const uint32_t kPeriod = 10U; // 10ms 주기 (100Hz)
	uint8_t state_local = OFF;
  for(;;)
  {
	  tick += kPeriod;
	  state_local = state_global;
	  // Central의 하트비트 검사
      if ((state_local != OFF) && (state_local != CENTRAL_DOWN) && (state_local != BROKEN_SELF))
      {
          if (HAL_GetTick() - last_zone_rx_time > 200) {
        	  state_local = CENTRAL_DOWN;
          }
      }
	  if(SPI2_ERR_FLAG){
		  CAN_SEND(ID_2_MOTOR_SPI_ERR);
	  }
	  // CAN 통신 중단 플래그 확인
	  switch(state_local){
      case BROKEN_SELF:
           HAL_CAN_Stop(&hcan);
           break;

      case EMERGENCY:
          if (osMessageQueueGet(canpedalQueueHandle, &rx_msg, NULL, 0) == osOK) {// spi2Task에서 큐에 삽입했으면!
              CAN_SEND(ID_2_MOTOR_STATUS);
          }
          CAN_SEND(ID_2_HB); // Heartbeat 전송
          break;
      case DRIVE:
      case PEDAL_ERR: // 페달 에러 시에도 중앙제어기에게 현재 모터 value랑 초음파값은 전해줘야함
          if (osMessageQueueGet(cansonicQueueHandle, &rx_msg, NULL, 0) == osOK) {// spi1Task에서 큐에 삽입했으면!
               CAN_SEND(ID_2_SONIC);
          }

          if (osMessageQueueGet(canpedalQueueHandle, &rx_msg, NULL, 0) == osOK) {// spi2Task에서 큐에 삽입했으면!
               CAN_SEND(ID_2_MOTOR_STATUS);
          }
          //fall through 씀
      case MOTOR_ERR:
      case READY:
          CAN_SEND(ID_2_HB);
          break;

      case CENTRAL_DOWN: // 중앙 다운에서 Zone2는 Zone1의 페달 데이터만 받을 뿐 아무것도 송신하지는 않는다.
    	  CAN_OFF_FILTER();
      case OFF:
      default:
          break;
	  }
	  osDelayUntil(tick); // 100Hz
  }
}


void spi2Task(void *argument)
{
	uint8_t seq = 0;
	uint8_t state_local = OFF;
	uint32_t tick = osKernelGetTickCount();
	const uint32_t kPeriod = 10U; // 10ms 주기 (100Hz)
	uint8_t SPI2_ERR_CNT = 0;
	uint8_t last_rx_seq = 0xFF;
  for(;;)
  {

	  tick += kPeriod;
	  state_local = state_global;
	  uint32_t result = osThreadFlagsWait(FLAG_MOTOR_SEND, osFlagsWaitAny, osWaitForever);
	  // 모터 버퍼 긁어와서 DMA 전용 버퍼에 저장
	    if(result & FLAG_MOTOR_SEND){ // CAN RX CALLBACK에서 들어왔다면
	    	bool frame_valid = true;
	    	if((state_local != DRIVE) && (state_local != CENTRAL_DOWN)) g_pedal = 0;// EMERGENCY, MOTOR_ERR, PEDAL_ERR 등 다 여기서 걸려서 g_pedal = 0으로 모터 정지.

	    	g_pedal = (g_pedal < 0) ? 0 : ((g_pedal > 3000) ? 3000 : g_pedal); // 랩 처리. 최대 3000까지만 듀티 허용
	    	// SPI DMA 보낼 패킷 구성
	    	pedal_tx.pkt.header = 0xAA;
	    	pedal_tx.pkt.data = g_pedal;
	    	pedal_tx.pkt.status = SPI2_ERR_FLAG? 0xFF : 0x00;
	    	pedal_tx.pkt.seq = seq++;
	    	pedal_tx.pkt.reserved = 0x00;
	    	pedal_tx.pkt.crc = crc_xor(pedal_tx.bytes, 6);
	    	pedal_tx.pkt.tail = 0xED;

	    	HAL_GPIO_WritePin(CS_MOTOR_GPIO_Port, CS_MOTOR_Pin, GPIO_PIN_RESET); // 이제 SPI 보낸다
	    	if(HAL_SPI_TransmitReceive_DMA(&hspi2, pedal_tx.bytes, pedal_rx.bytes, 8) == HAL_OK) { // 모터 드라이버 담당 MCU에게 전송
	    		uint32_t motor_res = osThreadFlagsWait(FLAG_MOTOR_DMA_COMPLETE, osFlagsWaitAny, 10); // 완료 대기
	    		    		if(motor_res == osFlagsErrorTimeout){ // 고장시
	    		    			HAL_SPI_DMAStop(&hspi2); // 멈추고
	    		    			hspi2.State = HAL_SPI_STATE_READY; // READY로 강제 변경해 RECOVERY 유도
	    		    			frame_valid = false;
	    		    		}
	    	}
	    	HAL_GPIO_WritePin(CS_MOTOR_GPIO_Port, CS_MOTOR_Pin, GPIO_PIN_SET); // SPI 다 보냈다

	    	// SPI 에러 처리 -> 하드웨어로 감지하지 못하는, 데이터 검증 및 상대방이 살아있는지 여부
	    	if(frame_valid){
	        	if(pedal_rx.pkt.header != 0xAA) frame_valid = false;
	        	if(pedal_rx.pkt.tail != 0xED) frame_valid = false;
	        	if(crc_xor((uint8_t *)&pedal_rx.pkt, 6) != pedal_rx.pkt.crc) frame_valid = false;
	    	}
	    	if(!frame_valid){
	    		if(++SPI2_ERR_CNT >= 20){ // 20번 연속으로 에러 뜨면
	    			if(state_global == DRIVE) SPI2_ERR_FLAG = 1;
	    		}
	    		continue;
	    	}

	    	if (pedal_rx.pkt.seq == last_rx_seq) {
	    	    if (++SPI2_ERR_CNT >= 20)
	    	    	if(state_global == DRIVE) SPI2_ERR_FLAG = 1;
	    	    continue;
	    	}

	    	// 이제부턴 정상 처리
	    	last_rx_seq = pedal_rx.pkt.seq;
	    	g_pedal_from_driver = pedal_rx.pkt.data;
	    	SPI2_ERR_CNT = 0;
	    	SPI2_ERR_FLAG = 0;
	    	Put_Latest_Data(canpedalQueueHandle, g_pedal); // 잘 보냈다. 큐 대신 플래그 세워도 되나 큰 오버헤드 없다고 판단해 구조적 통일성을 위함.
	    }
	    osDelayUntil(tick); // 100Hz
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
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
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
  HAL_GPIO_WritePin(GPIOA, CS_SONIC_Pin|CS_MOTOR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_SONIC_Pin CS_MOTOR_Pin */
  GPIO_InitStruct.Pin = CS_SONIC_Pin|CS_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


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
