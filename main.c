#include "main.h"
//#include <stdio.h>
#include <stdint.h>

#define USART_CLOCK     84000000    // 84MHz (APB2 클럭)
#define BAUD_RATE       115200
#define OVERSAMPLING    16

//void enable_processor_faults(void);
void Systemclock_Configuration(void);
void usart_init(void);
void usart_port_init(void);
void gpio_init(void);
void send_char(char data);
void send_string(char *str);
uint16_t calculate_brr_accurate(uint32_t apb_clock, uint32_t baud_rate);

int main(void)
{
    // 시스템 클록 설정
    Systemclock_Configuration();
    // GPIO 설정
    gpio_init();
    usart_port_init();
    usart_init();    
    //enable_processor_faults();
    while (1)
    {
        send_string("Baremetal USART1 update\r\n");
        for(int i = 0; i < 1000000; i++);
    }
}

void Systemclock_Configuration(void)
{
    // 기존 설정들...
    FLASH_ACR_ADDR |= (1 << 8) | (1 << 9) | (1 << 10);
    FLASH_ACR_ADDR &= ~(0x7 << 0);
    FLASH_ACR_ADDR |= (5 << 0);
    
    RCC_APB1ENR_ADDR |= (1 << 28);
    (*(volatile uint32_t*)0x40007000) |= (1 << 14);

    // HSE 활성화
    RCC_CR_ADDR |= (1 << 16);
    while(!(RCC_CR_ADDR & (1 << 17)));
    
    // PLL을 완전히 정지
    //RCC_CR_ADDR &= ~(1 << 24);  // PLL OFF
    //while(RCC_CR_ADDR & (1 << 25)); // PLL 완전 정지 대기
    
    RCC_PLLCFGR_ADDR &= ~(0x3F << 0); // PLLM 초기화
    RCC_PLLCFGR_ADDR &= ~(0xFF << 6); // PLLN 초기화
    RCC_PLLCFGR_ADDR &= ~(0x03 << 16); // PLLP 초기화
    RCC_PLLCFGR_ADDR &= ~(0x0F << 24); // PLLQ 초기화
    
    RCC_PLLCFGR_ADDR = (1 << 22) |  // HSE 소스
                       (25 << 0) |   // PLLM = 25
                       (336 << 6) |  // PLLN = 336
                       (0 << 16) |   // PLLP = 2
                       (7 << 24);    // PLLQ = 7

    // PLL 재시작
    RCC_CR_ADDR |= (1 << 24);
    while(!(RCC_CR_ADDR & (1 << 25)));
    
    // 나머지는 동일...
    RCC_CFGR_ADDR &= ~(0xF << 4);
    RCC_CFGR_ADDR |= (0 << 4);
    
    RCC_CFGR_ADDR &= ~(0x7 << 10);
    RCC_CFGR_ADDR |= (5 << 10);
    
    RCC_CFGR_ADDR &= ~(0x7 << 13);
    RCC_CFGR_ADDR |= (4 << 13);

    RCC_CFGR_ADDR &= ~(0x3 << 0);
    RCC_CFGR_ADDR |= (2 << 0);
    while((RCC_CFGR_ADDR & (0x3 << 2)) != (2 << 2));
}

void gpio_init(void)
{
    // Cortex-M4 에서 PORT RCC를 동작 시키려면 AHB1을 활성화 해야 합니다.
    // GPIO I RCC 동작
    RCC_AHB1ENR_ADDR |= (1 << 8) | (1 << 5);
    
    // GPIO I LED 동작
    GPIOI_MODER_ADDR &= ~(0x03 << 20);
    GPIOI_MODER_ADDR |= (0x01 << 20);     
    // output type register
    GPIOI_OTYPER_ADDR |= (0x00 << 10); // PORTI 10 push pull 
    // output speed register
    GPIOI_OSPEEDR_ADDR &= ~(0x03 << 20);  // high speed
    GPIOI_OSPEEDR_ADDR |= (0x02 << 20);
    // pull up pull down regisetr
    GPIOI_PUPDR_ADDR &= ~(0x03 << 20);     
    GPIOI_PUPDR_ADDR |= (0x00 << 20);     // No Pull up pull down

    // GPIO F LED 동작
    GPIOF_MODER_ADDR &= ~(0x03 << 14);
    GPIOF_MODER_ADDR |= (0x01 << 14);
    // output type register
    GPIOF_OTYPER_ADDR |= (0x00 << 7); // PORTF 7 push pull
    // output speed register
    GPIOF_OSPEEDR_ADDR &= ~(0x03 << 14);  // high speed
    GPIOF_OSPEEDR_ADDR |= (0x02 << 14);
    // pull up pull down regisetr
    GPIOF_PUPDR_ADDR &= ~(0x03 << 14);     
    GPIOF_PUPDR_ADDR |= (0x00 << 14);     // No Pull up pull down    
}

void usart_port_init(void){
    // USART1 clock enable
    RCC_APB2ENR_ADDR |= (1 << 4);
    // GPIOA RCC 활성화
    RCC_AHB1ENR_ADDR |= (1 << 0);

    // PA9(TX), PA10(RX)를 AF 모드로 설정
    GPIOA_MODER_ADDR &= ~((0x03 << 18) | (0x03 << 20));  // PA9, PA10 클리어
    GPIOA_MODER_ADDR |= ((0x02 << 18) | (0x02 << 20));   // AF 모드
    
    GPIOA_OTYPER_ADDR &= ~((1 << 9) | (1 << 10));  // Push-Pull
    
    // 출력 속도 설정 (PA9, PA10)
    GPIOA_OSPEEDR_ADDR &= ~((0x03 << 18) | (0x03 << 20));
    GPIOA_OSPEEDR_ADDR |= ((0x03 << 18) | (0x03 << 20)); // Very high speed

    // Pull-up/Pull-down 설정
    //GPIOA_PUPDR_ADDR &= ~((0x03 << 18) | (0x03 << 20));
    //GPIOA_PUPDR_ADDR |= ((0x00 << 18) | (0x00 << 20));   // No pull

    GPIOA_AFRH_ADDR &= ~((0x0F << 4) | (0x0F << 8));   // PA9, PA10 클리어
    GPIOA_AFRH_ADDR |= ((0x07 << 4) | (0x07 << 8));    // AF7 설정
}

uint16_t calculate_brr_accurate(uint32_t apb_clock, uint32_t baud_rate) {
    // Step 1: USART_DIV 계산 (고정소수점 연산 사용)
    uint32_t usart_div_x1000 = (apb_clock * 1000) / (OVERSAMPLING * baud_rate);
    
    // Step 2: 정수부 추출
    uint16_t mantissa = usart_div_x1000 / 1000;
    
    // Step 3: 소수부 계산 (1/16 단위)
    uint32_t remainder = usart_div_x1000 % 1000;
    uint16_t fraction = (remainder * 16 + 500) / 1000;  // 반올림 포함
    
    // Step 4: BRR 조합
    return (mantissa << 4) | (fraction & 0xF);
}

void usart_init(void){
    USART1_CR1_ADDR &= ~(1 << 13);
    
    USART1_CR1_ADDR = 0;
    USART1_CR2_ADDR = 0;
    USART1_CR3_ADDR = 0;
    
    //USART1 control register 2
    USART1_CR2_ADDR |= (0 << 12); // stop bit 1 
    //USART1 control register 1
    USART1_CR1_ADDR |= (0 << 12); // word length : start bit, 8 Data bit , n Stop bit
    USART1_CR1_ADDR |= (0 << 10); // Parity None
    USART1_CR1_ADDR |= (1 << 2);  // Receiver enable (RX)
    USART1_CR1_ADDR |= (1 << 3);  // Transmitter enable (TX)
    USART1_CR1_ADDR |= (0 << 15); // oversampling by 16
    //USART1 control register 3
    //USART1_CR3_ADDR |= (1 << 8);  // RTS enable
    //USART1_CR3_ADDR |= (1 << 9);  // CTS enable

    USART1_BRR_ADDR = 0x2D9;   // 84MHz / (16 × 115200) = 45.57 ≈ 46
    /*
    84MHz / (16 × 115200) = 45.57 (정수부)(Mantissa)
    45<<4 비트 왼쪽으로 시프트 연산
    45 * 2^4 = 45 * 16 = 720

    0.57 x 16(Oversampling) = 9.17 = 9 (소수부)
    BRR = 720 + 9 = 729 --> Hex (0x2D9)
    */

    USART1_CR1_ADDR |= (1 << 13); // USART1 enable
}

void send_char(char data){
    while(!(USART1_SR_ADDR & (1 << 7)));    //플래그 대기
    USART1_DR_ADDR = data;
}

void send_string(char *str){
    while(*str){
        send_char(*str);
        str++;
    }
}






#if 0
void enable_processor_faults(void)
{
    uint32_t *pSHCSR = (uint32_t*)0xE000ED24;

    *pSHCSR |= ( 1 << 16); 
	*pSHCSR |= ( 1 << 17); 
	*pSHCSR |= ( 1 << 18); 
}
#endif

