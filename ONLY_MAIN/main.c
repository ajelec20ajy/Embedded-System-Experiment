#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>

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

volatile bool zone1_down = false;
volatile bool zone1_sonic_down = false;
volatile bool zone2_down = false;
volatile bool zone2_sonic_down = false;
volatile bool zone3_down = false;
volatile bool zone3_accel_down = false;
volatile bool zone4_down = false;
volatile bool MOTOR_DOWN = false;
volatile bool PEDAL_DOWN = false;
volatile uint8_t err_cnt_1 = 0;
volatile uint8_t err_cnt_2 = 0;
volatile uint8_t err_cnt_3 = 0;
volatile uint8_t err_cnt_4 = 0;
volatile uint8_t last_rx_cnt1 = 0xFF;  
volatile uint8_t last_rx_cnt2 = 0xFF;
volatile uint8_t last_rx_cnt3 = 0xFF;
volatile uint8_t last_rx_cnt4 = 0xFF;


void setup_can0() { 
    // SocketCAN Start
    printf("can0 인터페이스 설정 중...\n");
    system("sudo ip link set can0 down 2>/dev/null");
    system("sudo ip link set can0 up type can bitrate 500000");
    printf("can0 활성화 완료!\n");
}

uint8_t calculate_checksum(uint8_t *data) {
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) sum += data[i];
    return sum;
}

uint64_t get_tick_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL +
           ts.tv_nsec / 1000000ULL;
}

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame tx_frame, rx_frame;

// 데이터 저장 변수
    int16_t latest_distance1 = 0; //z1
    int16_t latest_distance2 = 0; //z2
    uint64_t latest_distance1_arrived = 0;
    uint64_t latest_distance2_arrived = 0;
    int16_t latest_pedal = 0;
    uint8_t current_state = OFF;
    uint8_t tx_counter = 0;
    float ax, ay, az, acc = 1;
    uint8_t emergency_acc_cnt = 0;

// 하트비트 감시용 타임스탬프
    uint64_t last_hb_z1 = 0;
    uint64_t last_hb_z2 = 0;
    uint64_t last_hb_z3 = 0;
    uint64_t last_hb_z4 = 0;
    uint64_t last_print_time = 0;

    setup_can0();


    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    // CAN Filter 설정
    struct can_filter rfilter[12];

    rfilter[0].can_id  = ID_1_SONIC;           rfilter[0].can_mask  = CAN_SFF_MASK;
    rfilter[1].can_id  = ID_1_PEDAL_SPI_ERR;   rfilter[1].can_mask  = CAN_SFF_MASK;
    rfilter[2].can_id  = ID_1_HB;              rfilter[2].can_mask  = CAN_SFF_MASK;

    rfilter[3].can_id  = ID_2_SONIC;           rfilter[3].can_mask  = CAN_SFF_MASK;
    rfilter[4].can_id  = ID_2_MOTOR_STATUS;    rfilter[4].can_mask  = CAN_SFF_MASK;
    rfilter[5].can_id  = ID_2_MOTOR_SPI_ERR;   rfilter[5].can_mask  = CAN_SFF_MASK;
    rfilter[6].can_id  = ID_2_HB;              rfilter[6].can_mask  = CAN_SFF_MASK;

    rfilter[7].can_id  = ID_3_ACC;             rfilter[7].can_mask  = CAN_SFF_MASK;
    rfilter[8].can_id  = ID_3_HB;              rfilter[8].can_mask  = CAN_SFF_MASK;

    rfilter[9].can_id = ID_4_IR;               rfilter[9].can_mask = CAN_SFF_MASK;
    rfilter[10].can_id = ID_4_BTN;              rfilter[10].can_mask = CAN_SFF_MASK;
    rfilter[11].can_id = ID_4_HB;               rfilter[11].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    printf("모니터링 시작 (2초 주기로 출력합니다...)\n");
    uint8_t sum = 0; 
    while (1) {
        // 수신 루프
        while (read(s, &rx_frame, sizeof(struct can_frame)) > 0) {
            uint32_t canid = rx_frame.can_id & CAN_SFF_MASK;
            time_t now = time(NULL);

            switch (canid) {
                case ID_1_SONIC: 
                    sum = 0;
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_1++; continue;} // 체크섬검사
                    if(last_rx_cnt1 == rx_frame.data[6]) {err_cnt_1++; continue;} // 시퀀스 검사
                    if(err_cnt_1 > 10) zone1_down = true;
                    last_rx_cnt1 = rx_frame.data[6]; 
                    err_cnt_1 = 0;
                    zone1_down = false;
                    latest_distance1 = (rx_frame.data[1] << 8) | rx_frame.data[0]; 
                    latest_distance1_arrived = get_tick_ms();
                    break;

                case ID_1_PEDAL_SPI_ERR: PEDAL_DOWN = true; printf("pedal down\r\n"); break;

                case ID_2_SONIC: 
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_2++; continue;} // 체크섬검사
                    if(last_rx_cnt2 == rx_frame.data[6]) {err_cnt_2++; continue;} // 시퀀스 검사
                    if(err_cnt_2 > 10) zone2_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt2 = rx_frame.data[6]; 
                    err_cnt_2 = 0;
                    zone2_down = false;
                    latest_distance2 = (rx_frame.data[1] << 8) | rx_frame.data[0]; 
                    latest_distance2_arrived = get_tick_ms();
                    break;

                case ID_2_MOTOR_STATUS: 
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_2++; continue;} // 체크섬검사
                    if(last_rx_cnt2 == rx_frame.data[6]) {err_cnt_2++; continue;} // 시퀀스 검사
                    if(err_cnt_2 > 10) zone2_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt2 = rx_frame.data[6]; 
                    err_cnt_2 = 0;
                    zone2_down = false;
                    latest_pedal = (rx_frame.data[1] << 8) | rx_frame.data[0]; 
                    break;

                case ID_2_MOTOR_SPI_ERR: MOTOR_DOWN = true; printf("MOTOR DOWN\r\n"); break;

                case ID_3_ACC: 
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_3++; continue;} // 체크섬검사
                    if(last_rx_cnt3 == rx_frame.data[6]) {err_cnt_3++; continue;} // 시퀀스 검사
                    if(err_cnt_3 > 10) zone3_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt3 = rx_frame.data[6]; 
                    err_cnt_3 = 0;
                    zone3_down = false;
                    if((int16_t)((rx_frame.data[1] << 8) | rx_frame.data[0]) == 0xFFFF
                         && (int16_t)((rx_frame.data[3] << 8) | rx_frame.data[2]) == 0xFFFF
                         && (int16_t)((rx_frame.data[5] << 8) | rx_frame.data[4]) == 0xFFF) {
                            if(current_state == DRIVE) zone3_accel_down = true;// Zone3의 가속도 Down
                            continue;
                    }
                    zone3_accel_down = false;
                    ax = (int16_t)((rx_frame.data[1] << 8) | rx_frame.data[0]) / 100.0f;
                    ay = (int16_t)((rx_frame.data[3] << 8) | rx_frame.data[2]) / 100.0f;
                    az = (int16_t)((rx_frame.data[5] << 8) | rx_frame.data[4]) / 100.0f;
                    acc = sqrt((ax*ax)+(ay*ay)+(az*az)); if(acc == 0) acc = 1;
                    break;

                case ID_4_IR: 
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_4++; continue;} // 체크섬검사
                    if(last_rx_cnt4 == rx_frame.data[6]) {err_cnt_4++; continue;} // 시퀀스 검사
                    if(err_cnt_4 > 10) zone4_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt4 = rx_frame.data[6]; 
                    err_cnt_4 = 0;
                    zone4_down = false;
                    if(current_state == OFF) {
                        current_state = READY; 
                        uint64_t READ_START = get_tick_ms();
                        last_hb_z1 = last_hb_z2 = last_hb_z3 = last_hb_z4 = READ_START;
                        zone1_down = zone2_down = zone3_down = zone4_down = false;
                    }
                    break;

                case ID_4_BTN:
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_4++; continue;} // 체크섬검사
                    if(last_rx_cnt4 == rx_frame.data[6]) {err_cnt_4++; continue;} // 시퀀스 검사
                    if(err_cnt_4 > 10) zone4_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt4 = rx_frame.data[6]; 
                    err_cnt_4 = 0;
                    zone4_down = false;
                    if(current_state == READY) current_state = DRIVE;             
                    latest_distance1_arrived = get_tick_ms();
                    latest_distance2_arrived = get_tick_ms();
                    break;

                case ID_1_HB: 
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_1++; continue;} // 체크섬검사
                    if(last_rx_cnt1 == rx_frame.data[6]) {err_cnt_1++; continue;} // 시퀀스 검사
                    if(err_cnt_1 > 10) zone1_down = true;
                    last_rx_cnt1 = rx_frame.data[6]; 
                    err_cnt_1 = 0;
                    last_hb_z1 = get_tick_ms();      
                    zone1_down = false;
                    break;

                case ID_2_HB:
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_2++; continue;} // 체크섬검사
                    if(last_rx_cnt2 == rx_frame.data[6]) {err_cnt_2++; continue;} // 시퀀스 검사
                    if(err_cnt_2 > 10) zone2_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt2 = rx_frame.data[6]; 
                    err_cnt_2 = 0;
                    last_hb_z2 = get_tick_ms();
                    zone2_down = false;
                    break;

                case ID_3_HB:
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_3++; continue;} // 체크섬검사
                    if(last_rx_cnt3 == rx_frame.data[6]) {err_cnt_3++; continue;} // 시퀀스 검사
                    if(err_cnt_3 > 10) zone3_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt3 = rx_frame.data[6]; 
                    err_cnt_3 = 0;
                    last_hb_z3 = get_tick_ms(); 
                    zone3_down = false;
                    break;

                case ID_4_HB:
                    sum = 0; 
                    for(int i = 0; i<7; i++){
                        sum += rx_frame.data[i];
                    }
                    if(rx_frame.data[7] != sum) {err_cnt_4++; continue;} // 체크섬검사
                    if(last_rx_cnt4 == rx_frame.data[6]) {err_cnt_4++; continue;} // 시퀀스 검사
                    if(err_cnt_4 > 10) zone4_down = true; //10번 연속 패킷 에러면
                    last_rx_cnt4 = rx_frame.data[6]; 
                    err_cnt_4 = 0;
                    
                    last_hb_z4 = get_tick_ms(); 
                    zone4_down = false;
                    break;

                default: break; // 정의되지 않은 ID는 무시
            }
        }
        uint64_t test_now = get_tick_ms();
        if(current_state != OFF && test_now - last_hb_z1 > 300) zone1_down = true;
        if(current_state != OFF && test_now - last_hb_z2 > 300) zone2_down = true;
        if(current_state != OFF && test_now - last_hb_z3 > 300) zone3_down = true;
        if(current_state != OFF && test_now - last_hb_z4 > 300) zone4_down = true;

        if(get_tick_ms() - latest_distance1_arrived > 1000) {if(current_state == DRIVE) zone1_sonic_down = true;} else zone1_sonic_down = false;
        if(get_tick_ms() - latest_distance2_arrived > 1000) {if(current_state == DRIVE) zone2_sonic_down = true;} else zone2_sonic_down = false;

        if((current_state != OFF) && zone1_down){ // ZONE1 떨어져나가면 이머전시 전파(페달 조작없으니깐 멈춰야됌)
            current_state = EMERGENCY;
        }
        if((current_state != OFF) && zone2_down){
            current_state = EMERGENCY;
        }
        if((current_state != OFF) && MOTOR_DOWN){
            current_state = MOTOR_ERR;
        }
        if((current_state != OFF) && PEDAL_DOWN){
            current_state = PEDAL_ERR;
        }

        if(current_state == EMERGENCY){ // 이머전시에서 레디로 복귀하는 조건(가속도 = 1G)
            if(fabs(acc - 1.0f) <= 0.1) emergency_acc_cnt++;
            else emergency_acc_cnt = 0;
            if(emergency_acc_cnt >= 20 && !zone1_down && !zone2_down && !zone3_down && !zone4_down && !PEDAL_DOWN && !MOTOR_DOWN) {
                current_state = READY; 
                latest_distance1=latest_distance2=0; 
                emergency_acc_cnt = 0;                    
                latest_distance1_arrived = get_tick_ms();
                latest_distance2_arrived = get_tick_ms();
            }
        }
        // CENTRAL DOWN 로직
        if(zone1_down && zone2_down && zone3_down && zone4_down && current_state != OFF) current_state = CENTRAL_DOWN;

        // emergency 로직
        if ((!zone1_sonic_down) && current_state == DRIVE && latest_distance1 > 0 && latest_distance1 <= 3) {
            current_state = EMERGENCY;
            printf("\n[!!!] EMERGENCY STOP: Distance1 detected at %d cm!\n", latest_distance1);
        }

        if ((!zone2_sonic_down) && current_state == DRIVE && latest_distance2 > 0 && latest_distance2 <= 3) {
            current_state = EMERGENCY;
            printf("\n[!!!] EMERGENCY STOP: Distance2 detected at %d cm!\n", latest_distance2);
        }

        if((!zone3_accel_down) && current_state == DRIVE && (fabs(acc - 1) > 2)){
            current_state = EMERGENCY;
            printf("\n[!!!] EMERGENCY STOP : Collision Accelaration detected %fg!\n", fabs(acc));
        }
        // 2초 주기 상태 출력
        uint64_t now = get_tick_ms();
        if (now - last_print_time >= 2000) {
            printf("\n--- [%ld] 시스템 상태 요약 ---\n", now);
            // 상태 문자열 처리를 switch나 다중 조건으로 깔끔하게 변경
             const char* mode_name;
            if(current_state == CENTRAL_DOWN) mode_name = "CENTRAL_DOWN";
            else if (current_state == DRIVE) mode_name = "DRIVE";
            else if (current_state == READY) mode_name = "READY";
            else if (current_state == EMERGENCY) mode_name = "EMERGENCY (STOPPED)"; // 이 부분 추가!
            else if (current_state == MOTOR_ERR) mode_name = "MOTOR_ERR";
            else if (current_state == PEDAL_ERR) mode_name = "PEDAL_ERR";
            else if (current_state == CENTRAL_DOWN) mode_name = "CENTRAL_DOWN";
            else if (current_state == OFF) mode_name = "OFF";
            if(zone1_down) printf("Zone1 Down 감지 !! Emergency\r\n");
            if(zone2_down) printf("Zone2 Down 감지 !! Emergency\r\n");
            printf("현재 모드: %s\n", mode_name);
            printf("초음파 거리: Z1: %4d cm | Z2: %4d cm\n", latest_distance1, latest_distance2);
            printf("초음파 상태: Z1: %d | Z2 : %d \n",zone1_sonic_down, zone2_sonic_down);
            printf("페달 수신값(0x110): %d\n", latest_pedal);
            printf("가속도 수신값: %f\n", acc);
            
            // 하트비트 기반 생존 여부
            printf("노드 상태: Zone1[%s] Zone2[%s] Zone3[%s] Zone4[%s]\n",
                   (zone1_down) ? "DEAD" : "ALIVE",
                   (zone2_down) ? "DEAD" : "ALIVE",
                   (zone3_down) ? "DEAD" : "ALIVE",
                   (zone4_down) ? "DEAD" : "ALIVE");
            printf("------------------------------\n");
            
            last_print_time = now;
        }

        // 중앙 제어기 하트비트 송신 (100Hz)
        // 현 상태에 따라 송신 id 결정
        
        if (current_state == EMERGENCY) {
            tx_frame.can_id = ID_CENTRAL_EMERGENCY; // 0x002
        } else {
            tx_frame.can_id = ID_CENTRAL_STATUS;    // 0x004
        }
        if(current_state == CENTRAL_DOWN){
        }
        else{
            tx_frame.can_dlc = 8;
            memset(tx_frame.data, 0, 8);
        
            //if(current_state == DRIVE) tx_counter = 0; // (2) CENTRAL_DOWN2 의도적으로 드라이브상태에서 카운터 망가뜨리기
            tx_frame.data[0] = current_state; 
            tx_frame.data[6] = tx_counter++;
            tx_frame.data[7] = calculate_checksum(tx_frame.data);
            write(s, &tx_frame, sizeof(struct can_frame));
        }
        
        usleep(10000); // 10ms
    }

    close(s);
    return 0;
}
