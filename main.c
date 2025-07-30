#include "main.h"
//#include <stdio.h>
#include <stdint.h>

typedef enum {
    TIMEOUT_OK = 0,
    TIMEOUT_ERROR = 1
} TimeoutStatus_t;

#define MAX_DELAY 0xFFFFFFFFU

#define USART_CLOCK     84000000    // 84MHz (APB2 클럭)
#define BAUD_RATE       115200
#define OVERSAMPLING    16

#define UNUSED(X) (void)X

#define CMD_RDID      0x9F
#define DUMMY_BYTE    0xA5

// system tick counter
volatile uint32_t system_tick = 0;

//void enable_processor_faults(void);
void Systemclock_Configuration(void);
void usart_init(void);
void usart_port_init(void);
void gpio_init(void);
void send_char(char data);
void send_digit(uint8_t digit);
void send_number(uint32_t number);
void send_hex(uint32_t number);

//void send_format(char *format, uint32_t value);
//void send_format2(char *format, uint32_t value1, uint32_t value2);

void send_string(char *str);
void spi_port_init(void);
void spi_flash_init(void);
void sf_ReadInfo(void);
uint32_t sf_ReadID(void);
static uint8_t sf_SendByte(uint8_t _ucValue);
void SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void SysTick_Init(uint32_t system_clock_hz);
uint32_t GetTick(void);

// SPI1
#define SF_CS_LOW()     (GPIOF_ODR_ADDR &= ~(1 << 8))
#define SF_CS_HIGH()     (GPIOF_ODR_ADDR |= (1 << 8))

__attribute__((always_inline)) static inline void __enable_irq(void){
    __asm volatile ("cpsie i" : : : "memory");
}

TimeoutStatus_t CheckTimeout(uint32_t tickstart, uint32_t timeout){
    if (((GetTick() - tickstart) >= timeout) && (timeout != MAX_DELAY)) {
        return TIMEOUT_ERROR;
    }
    
    if (timeout == 0U) {
        return TIMEOUT_ERROR;
    }
    return TIMEOUT_OK;
}

int main(void)
{
    uint32_t counter = 0;
    // 시스템 클록 설정
    Systemclock_Configuration();
    
    SysTick_Init(168000000); // 168MHz 시스템 클록
    
    __enable_irq();
    
    // GPIO 설정
    gpio_init();
    usart_port_init();
    usart_init();    
    
    spi_port_init();
    spi_flash_init();        
    
    //enable_processor_faults();
    while (1)
    {
        #if 0
        //send_string("Baremetal USART1 update\r\n");
        uint32_t tickstart = GetTick();
        
        send_string("Tick: ");
        send_number(tickstart);
        send_string(", Counter: ");
        send_number(counter);
        send_string("\r\n");
        
        counter++;
        
        uint32_t start = GetTick();
        while ((GetTick() - start) < 1000) {}
        #endif
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

void send_digit(uint8_t digit) {
    send_char('0' + digit);
}

void send_number(uint32_t number) {
    char buffer[12];  // 32비트 최대값: 4294967295 (10자리) + null
    int i = 0;
    
    // 0인 경우 특별 처리
    if (number == 0) {
        send_char('0');
        return;
    }
    
    // 숫자를 역순으로 buffer에 저장
    while (number > 0) {
        buffer[i++] = '0' + (number % 10);
        number /= 10;
    }
    
    // 역순으로 저장된 것을 정순으로 출력
    for (int j = i - 1; j >= 0; j--) {
        send_char(buffer[j]);
    }
}

// 16진수로 출력 (디버깅용)
void send_hex(uint32_t number) {
    char hex_chars[] = "0123456789ABCDEF";
    send_string("0x");
    
    for (int i = 28; i >= 0; i -= 4) {
        send_char(hex_chars[(number >> i) & 0xF]);
    }
}

#if 0
void send_format(char *format, uint32_t value) {
    while (*format) {
        if (*format == '%' && *(format + 1) == 'd') {
            send_number(value);
            format += 2;  // %d 건너뛰기
        } else if (*format == '%' && *(format + 1) == 'x') {
            send_hex(value);
            format += 2;  // %x 건너뛰기
        } else {
            send_char(*format);
            format++;
        }
    }
}

void send_format2(char *format, uint32_t value1, uint32_t value2) {
    int param_count = 0;
    while (*format) {
        if (*format == '%' && *(format + 1) == 'd') {
            if (param_count == 0) send_number(value1);
            else if (param_count == 1) send_number(value2);
            param_count++;
            format += 2;
        } else {
            send_char(*format);
            format++;
        }
    }
}
#endif

void send_string(char *str){
    while(*str){
        send_char(*str);
        str++;
    }
}

void spi_port_init(void)
{
    // GPIO B , GPIO F RCC enable
    RCC_AHB1ENR_ADDR |= ((1 << 1) | (1 << 5));
    // SPI1 clock enable
    RCC_APB2ENR_ADDR |= (1 << 12);
    // AF 설정 (SPI 1)
    // GPIOB 3,4,5
    // GPIOB MODE , AF Mode
    GPIOB_MODER_ADDR &= ~((0x3 << 6) | (0x3 << 8) | (0x3 << 10));
    GPIOB_MODER_ADDR  |= ((0x2 << 6) | (0x2 << 8) | (0x2 << 10));
    // GPIOB SPEED
    GPIOB_OSPEEDR_ADDR &= ~((0x3 << 6) | (0x3 << 8) | (0x3 << 10));
    GPIOB_OSPEEDR_ADDR |= ((0x2 << 6) |  (0x2 << 8) | (0x2 << 10)); 
    // GPIOB PULL DOWN   
    GPIOB_PUPDR_ADDR &= ~((0x3 << 6) | (0x3 << 8) | (0x3 << 10));
    GPIOB_PUPDR_ADDR |= ((0x2 << 6) |  (0x2 << 8) | (0x2 << 10));
    // GPIOB AF setting. SPI1
    GPIOB_AFRL_ADDR &= ~((0xF << 12) | ( 0xF << 16) | ( 0xF << 20));
    GPIOB_AFRL_ADDR |= ((0x5 << 12) | ( 0x5 << 16) | ( 0x5 << 20));

    SF_CS_HIGH();
    // GPIOF output mode
    GPIOF_MODER_ADDR &= ~(0x3 << 16);
    GPIOF_MODER_ADDR |= (0x1 << 16);
    // GPIOF SPEED
    GPIOF_OSPEEDR_ADDR &= ~(0x3 << 16); 
    GPIOF_OSPEEDR_ADDR |= (0x2 << 16);
    // GPIOF NO FULL
    GPIOF_PUPDR_ADDR &= ~(0x3 << 16);
    //GPIOF_PUPDR_ADDR |= (0x0 << 16);
}

void spi_flash_init(void)
{
    SPI1_CR1_ADDR |= ((0x1 << 0)  |              // The second clock transition is the first data capture edge
                      (0x1 << 1)  |              // CK to 1 when idle 
                      (0x1 << 2)  |              // Master configuration
                      (0x1 << 6)  |              // SPI enable , Peripheral enabled
                      (0x0 << 7)  |              // MSB transmitted first
                      (0x1 << 8)  |              // SSI
                      (0x1 << 9)  |              // Software slave management enabled
                      (0x0 << 10) |              // Full duplex
                      (0x0 << 11) |              // 8-bit data frame format is selected for transmission/reception
                      (0x0 << 13) |              // CRC calculation disabled
                      (0x0 << 15));              // BIDIMODE = 0 (2-line unidirectional)  

    SPI1_CR1_ADDR &= ~(0x7 << 3);
    SPI1_CR1_ADDR |=  (1 << 3);                   // fpclk / 4
    
    //SS output enable
    SPI1_CR2_ADDR |= (0x1 << 2);                  // SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment.??
    //Frame format
    SPI1_CR2_ADDR &= ~(0x1 << 4);                 // FRF = 0, Standard SPI mode

    sf_ReadInfo();
}

uint32_t sf_ReadID(void)
{
    uint32_t uiID;
	uint8_t id1, id2, id3;
    
    SF_CS_LOW();
    sf_SendByte(CMD_RDID);
    id1 = sf_SendByte(DUMMY_BYTE);
    id2 = sf_SendByte(DUMMY_BYTE);
    id3 = sf_SendByte(DUMMY_BYTE);
    SF_CS_HIGH();

    uiID = ((uint32_t)id1 << 16) | ((uint32_t)id2 << 8) | id3;

    return uiID; 
}

static uint8_t sf_SendByte(uint8_t _ucValue)
{
    uint8_t rxData = 0;
    SPI_TransmitReceive(&_ucValue, &rxData, 1);
    return rxData;
}

#if 1
void SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    uint8_t * pRxBuffPtr = (uint8_t *)pRxData;
    uint16_t RxXferCount = Size;
    //uint16_t RxXferSize = Size;
    uint8_t * pTxBuffPtr = (uint8_t *)pTxData;
    uint16_t TxXferCount = Size;
    //uint16_t TxXferSize = Size;
    uint32_t txallowed = 1U;
    
    SPI1_DR_ADDR = *pTxBuffPtr;
    pTxBuffPtr += sizeof(uint8_t);
    TxXferCount--;

    while((TxXferCount > 0U) || (RxXferCount > 0U))
    {
        if((SPI1_SR_ADDR & (0x1 << 1)) && (TxXferCount > 0U) && (txallowed == 1U)){
            SPI1_DR_ADDR = *pTxBuffPtr;
            pTxBuffPtr++;
            TxXferCount--;
            txallowed = 0U;
        }
        if((SPI1_SR_ADDR & (0x1 << 0)) && (RxXferCount > 0U))
        {
            *pRxBuffPtr = SPI1_DR_ADDR;
            pRxBuffPtr++;
            RxXferCount--;
            txallowed = 1U;
        }
    }
    if(SPI1_CR1_ADDR & (0x0 << 10)){
        uint32_t tmpreg_ovr = 0x00U;
        do{
            tmpreg_ovr = SPI1_DR_ADDR;
            tmpreg_ovr = SPI1_SR_ADDR;
            UNUSED(tmpreg_ovr);
        }while(0U);
    }    
}
#endif

void sf_ReadInfo(void){
    uint32_t ChipID = sf_ReadID();
    send_hex(ChipID);
}

void SysTick_Init(uint32_t system_clock_hz){
    SysTick_LOAD = (system_clock_hz / 1000) - 1;
    SysTick_VAL = 0;   
    SysTick_CTRL |= (0x1 << 0) |            // counter enable
                    (0x1 << 1) |            // Counting down to zero to asserts the SysTick exception request.  
                    (0x1 << 2) ;            // Processor clock (AHB)    
}

void SysTick_Handler(void)
{
    system_tick++;
}

uint32_t GetTick(void) {
    return system_tick;
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

