#ifndef MAIN_H_
#define MAIN_H_

// RCC (Reset and Clock Control)
/*Table 1. STM32F4xx register boundary addresses (continued)*/
#define RCC_BASE         0x40023800UL

#define FLASH_ADD_BASE   0x40023C00UL

// RCC  레지스터 주소
// RCC Clock Control Register Address
#define RCC_CR_ADDR      (*(volatile uint32_t*)(RCC_BASE + 0x00))     // Register map 에 offset 값을 더함 = 레지스터 접근
// RCC PLL Configuration Register Address
#define RCC_PLLCFGR_ADDR (*(volatile uint32_t*)(RCC_BASE + 0x04))
// RCC clock configuration register (RCC_CFGR)
#define RCC_CFGR_ADDR    (*(volatile uint32_t*)(RCC_BASE + 0x08))
// RCC clock interrupt register (RCC_CIR)
#define RCC_CIR_ADDR     (*(volatile uint32_t*)(RCC_BASE + 0x0C))

#define RCC_AHB1ENR_ADDR  (*(volatile uint32_t*)(RCC_BASE + 0x30)) 

#define RCC_APB1ENR_ADDR    (*(volatile uint32_t*)(RCC_BASE + 0x40))
#define FLASH_ACR_ADDR      (*(volatile uint32_t*)(FLASH_ADD_BASE + 0x00))

#define GPIOI_BASE_ADDR      0x40022000UL
#define GPIOI_MODER_ADDR      (*(volatile uint32_t*)(GPIOI_BASE_ADDR + 0x00))
#define GPIOI_OTYPER_ADDR      (*(volatile uint32_t*)(GPIOI_BASE_ADDR + 0x04))
#define GPIOI_OSPEEDR_ADDR      (*(volatile uint32_t*)(GPIOI_BASE_ADDR + 0x08))
#define GPIOI_PUPDR_ADDR      (*(volatile uint32_t*)(GPIOI_BASE_ADDR + 0x0C))

#define GPIOF_BASE_ADDR      0x40021400UL
#define GPIOF_MODER_ADDR      (*(volatile uint32_t*)(GPIOF_BASE_ADDR + 0x00))
#define GPIOF_OTYPER_ADDR      (*(volatile uint32_t*)(GPIOF_BASE_ADDR + 0x04))
#define GPIOF_OSPEEDR_ADDR      (*(volatile uint32_t*)(GPIOF_BASE_ADDR + 0x08))
#define GPIOF_PUPDR_ADDR      (*(volatile uint32_t*)(GPIOF_BASE_ADDR + 0x0C))






#define GPIOI_BSRR_ADDR      (*(volatile uint32_t*)(GPIOI_BASE_ADDR + 0x18))
#define GPIOF_BSRR_ADDR      (*(volatile uint32_t*)(GPIOF_BASE_ADDR + 0x18))








#endif