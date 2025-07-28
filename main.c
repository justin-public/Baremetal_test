#include "main.h"
//#include <stdio.h>
#include <stdint.h>

//void enable_processor_faults(void);
void Systemclock_Configuration(void);
void gpio_init(void);

int main(void)
{
    // 시스템 클록 설정
    Systemclock_Configuration();
    // GPIO 설정
    gpio_init();
    
    //enable_processor_faults();
    while (1)
    {
        GPIOI_BSRR_ADDR = (0 << 10);
        GPIOF_BSRR_ADDR = (0 << 7);
    }
}

void Systemclock_Configuration(void)
{
    // Flash 영역 최적화
    // 즉시 성능 향상
    // 코드 실행 가속화
    // 클럭 변경 과정 빠르게 
    FLASH_ACR_ADDR |= (1 << 8);     // 프리패치 활성화
    FLASH_ACR_ADDR |= (1 << 9);     // 명령 캐시 메모리 동작 활성화
    FLASH_ACR_ADDR |= (1 << 10);    // 데이터 캐시 메모리 동작 활성화
    
    // 플래시 메모리 액세스 시간을 제어
    // 이 비트들은 CPU 클럭 주기와 플래시 메모리 접근 시간의 비율을 나타냅니다.
    FLASH_ACR_ADDR &= ~(0x7 << 0);
    FLASH_ACR_ADDR |= (5 << 0);
    
    // 내부 레귤레이터 출력 전압 구성
    // VOS (Voltage scaling output)
    // 이비트는 장치가 최대 주파수에서 작동하지 않을때 성능과 전력 소비사이의 균형을 맞추기 위해 주 내부 전압
    // 레귤레이터 출력 전압을 제어 
    RCC_APB1ENR_ADDR |= (1 << 28);   // PWR_ON
    (*(volatile uint32_t*)0x40007000) |= (1 << 14); // 1 리셋시 기본 값

    // RCC 초기화
    // RCC HSE ON
    RCC_CR_ADDR |= (1 << 16);
    while(!(RCC_CR_ADDR & (1 << 17))); // HSERDY 대기 (25MHz HSE 안정화)
    
    // PLL SOURCE , HSE Oscillator
    // PLL 설정 (PLL 켜지전에 !)
    RCC_PLLCFGR_ADDR |= (1 << 22);
    // PLLM Value
    RCC_PLLCFGR_ADDR |= (25 << 0);
    // PLLN Value
    RCC_PLLCFGR_ADDR |= (336 << 6);
    // PLLP Value
    RCC_PLLCFGR_ADDR |= (2 << 16);
    // PLLQ Value
    RCC_PLLCFGR_ADDR |= (7 << 24);

    // PLL ON
    RCC_CR_ADDR |= (1 << 24);
    while(!(RCC_CR_ADDR & (1 << 25))); // PLLRDY 대기 (PLL 락 완료)

    //RCC_CFGR_ADDR &= ~(0x3 << 0);  // SW 비트 클리어
    //RCC_CFGR_ADDR |= (2 << 0);     // PLL을 시스템 클럭으로 선택
    //while((RCC_CFGR_ADDR & (0x3 << 2)) != (2 << 2)); // SWS 비트로 전환 완료 대기
    
    // CPU, AHB 및  APB 버스 클럭 초기화 , 다중 비트를 컨트롤할 경우는 클리어가 필요함(안정성)
    // AHB prescaler
    RCC_CFGR_ADDR &= ~(0xF << 4);
    RCC_CFGR_ADDR |= (0 << 4);     // 168MHz
    
    // APB1 Low speed prescaler
    RCC_CFGR_ADDR &= ~(0x7 << 10);
    RCC_CFGR_ADDR |= (5 << 10);         // HCLK clock divider 4   
    
    // APB2 HIGH speed prescaler
    RCC_CFGR_ADDR &= ~(0x7 << 13);       
    RCC_CFGR_ADDR |= (4 << 13);         // HCLK clock divider 2

    RCC_CFGR_ADDR &= ~(0x3 << 0);  // SW 비트 클리어
    RCC_CFGR_ADDR |= (2 << 0);     // PLL을 시스템 클럭으로 선택
    while((RCC_CFGR_ADDR & (0x3 << 2)) != (2 << 2)); // SWS 비트로 전환 완료 대기
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


#if 0
void enable_processor_faults(void)
{
    uint32_t *pSHCSR = (uint32_t*)0xE000ED24;

    *pSHCSR |= ( 1 << 16); 
	*pSHCSR |= ( 1 << 17); 
	*pSHCSR |= ( 1 << 18); 
}
#endif

