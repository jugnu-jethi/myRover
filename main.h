#include "stm32f4xx.h"
#include <stdio.h>



#define SYSTICK_ENABLE
#define WATCHDOG_ENABLE
//#define IO_COMPENSATION_CELL_ENABLE



#define REVISION_Z (0x1001U << DBGMCU_IDCODE_REV_ID_Pos)
#define FALSE (0U)
#define TRUE (!FALSE)
#define TIMED_OUT (0U)
#define CONFIG_TIMEOUT_DURATION (0XFFFFFFFFU)
#define PLLM_DIV_4 ((4U) << RCC_PLLCFGR_PLLM_Pos)
#define PLLM_DIV_8 ((8U) << RCC_PLLCFGR_PLLM_Pos)
#define PLLN_MUL_336 ((336U) << RCC_PLLCFGR_PLLN_Pos)
#define PLLP_DIV_2 ((0U) << RCC_PLLCFGR_PLLP_Pos)
#define PLLP_DIV_4 ((1U) << RCC_PLLCFGR_PLLP_Pos)
#define PLLQ_DIV_7 ((7U) << RCC_PLLCFGR_PLLQ_Pos)
#define PLLQ_DIV_14 ((14U) << RCC_PLLCFGR_PLLQ_Pos)
#define IWDG_UNLOCK_KEY (0X5555U)
#define IWDG_START_KEY (0XCCCCU)
#define IWDG_RELOAD_KEY (0XAAAAU)
#define IWDG_PRESCALER_256 (7U)
#define IWDG_RELOAD_VALUE (4095U)
#define TIMER4_CR1_RESET (0U)
#define TIMER4_CR2_RESET TIMER4_CR1_RESET
#define TIMER4_CCMR1_RESET TIMER4_CR2_RESET
#define TIMER4_CCMR2_RESET TIMER4_CCMR1_RESET
#define TIMER4_CCER_RESET TIMER4_CCMR2_RESET
#define TIMER4_PSC_RESET TIMER4_CCMR2_RESET
#define ADC1_CR1_RESET (0U)
#define ADC1_CR2_RESET ADC1_CR1_RESET
#define FREQ_AN_MSEC (1000U)
#define MSEC_CALIBRATION_FACTOR (20U)
#define TIMER4_PSC_VALUE TIMER4_PSC_RESET
#define DRV8833_PWM_FREQUENCY (20000U)
#define TIMER4_PWM_PERIOD (84000000U/((TIMER4_PSC_VALUE + 1U) * DRV8833_PWM_FREQUENCY))



volatile uint32_t Delay = 0;
volatile uint32_t myTaskFlags = 0;
volatile IWDG_TypeDef *myIWatchDog = IWDG;
volatile EXTI_TypeDef *myEXTI = EXTI;
volatile ADC_TypeDef *myADC1 = ADC1;
volatile ADC_Common_TypeDef *ADCCommonControl = ADC;
volatile GPIO_TypeDef *myPortD = GPIOD;



void msec_Delay(uint32_t);
// Couldn't use "inline" keyword due to linker's undefined symbol error
// REF: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka15831.html
// REF: https://community.arm.com/tools/f/discussions/4891/inline-function-attribute-causes-undefined-symbol-linking-error
void Manage_Timeout(uint32_t) __attribute__((always_inline));
