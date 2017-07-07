#include "stm32f4xx.h"
#include <stdio.h>



//#define SYSTICK_ENABLE
//#define WATCHDOG_ENABLE



#define FALSE (0)
#define TRUE !(FALSE)
#define TIMED_OUT (0)
#define CONFIG_TIMEOUT_DURATION (0XFFFFFFFF)
#define PLLM_DIV_4 ((4U) << RCC_PLLCFGR_PLLM_Pos)
#define PLLM_DIV_8 ((8U) << RCC_PLLCFGR_PLLM_Pos)
#define PLLN_MUL_336 ((336U) << RCC_PLLCFGR_PLLN_Pos)
#define PLLP_DIV_2 ((0U) << RCC_PLLCFGR_PLLP_Pos)
#define PLLP_DIV_4 ((1U) << RCC_PLLCFGR_PLLP_Pos)
#define PLLQ_DIV_7 ((7U) << RCC_PLLCFGR_PLLQ_Pos)
#define PLLQ_DIV_14 ((14U) << RCC_PLLCFGR_PLLQ_Pos)
#define IWDG_UNLOCK_KEY (0X5555)
#define IWDG_START_KEY (0XCCCC)
#define IWDG_RELOAD_KEY (0XAAAA)
#define IWDG_PRESCALER_256 (7)
#define TIMER4_CR1_RESET (0)
#define TIMER4_CR2_RESET TIMER4_CR1_RESET
#define TIMER4_CCMR1_RESET TIMER4_CR2_RESET
#define TIMER4_CCMR2_RESET TIMER4_CCMR1_RESET
#define TIMER4_CCER_RESET TIMER4_CCMR2_RESET
#define TIMER4_PSC_RESET TIMER4_CCMR2_RESET
#define FREQ_AN_MSEC (1000)
#define MSEC_CALIBRATION_FACTOR (20)



volatile uint32_t Delay = 0;
volatile IWDG_TypeDef *myIWatchDog = IWDG;



void msec_Delay(uint32_t);
// Couldn't use "inline" keyword due to linker's undefined symbol error
// REF: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka15831.html
// REF: https://community.arm.com/tools/f/discussions/4891/inline-function-attribute-causes-undefined-symbol-linking-error
void Manage_Timeout(uint32_t) __attribute__((always_inline));



int main(){
	
	uint32_t timeout = 0;
	RCC_TypeDef *myRCC = RCC;
	GPIO_TypeDef *myPortD = GPIOD;
	GPIO_TypeDef *myPortB = GPIOB;
	FLASH_TypeDef *myFlash = FLASH;
	TIM_TypeDef *myTimer4 = TIM4;
	
	
	// Set flash wait states to 5
	// Reference Manual: Section 3.5.1
	myFlash->ACR &= ~(FLASH_ACR_LATENCY);
	myFlash->ACR |= FLASH_ACR_LATENCY_5WS;
	timeout = CONFIG_TIMEOUT_DURATION;
	//while(FLASH_ACR_LATENCY_5WS != (myFlash->ACR & FLASH_ACR_LATENCY));
	while(FLASH_ACR_LATENCY_5WS != (myFlash->ACR & FLASH_ACR_LATENCY)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
	// Enable prefetch; Enable Instruction Cache; Enable Data Cache
	myFlash->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	
	// Enable HSE oscillator source i.e. 8Mhz crystal
	myRCC->CR &= ~(RCC_CR_HSEBYP);
	myRCC->CR |= RCC_CR_HSEON;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(FALSE == (myRCC->CR & RCC_CR_HSERDY)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
	
	// Set-up PLL configuration
	// PLLM = 8, PLLN = 336, PLLP = 2, PLLQ = 7
	// fPLLIN = 8Mhz/8 = 1Mhz; fVCOOUT = 1Mhz * 336 = 336Mhz; fPLLOUT = 336Mhz/2 = 168Mhz
	// fUSBOTG = 336Mhz/7 = 48Mhz
	myRCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ);
	myRCC->PLLCFGR |= PLLM_DIV_8 | PLLN_MUL_336 | PLLP_DIV_2 | PLLQ_DIV_7 | RCC_PLLCFGR_PLLSRC_HSE;
	
	// Set-up AHB@168Mhz; APB1@42Mhz; APB2@84Mhz
	myRCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	myRCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
	
	// Enable PLL
	myRCC->CR |= RCC_CR_PLLON;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(FALSE == (myRCC->CR & RCC_CR_PLLRDY)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
	
	// Switch system clock to PLL source
	myRCC->CR &= ~(RCC_CFGR_SW);
	myRCC->CFGR |= RCC_CFGR_SW_PLL;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(RCC_CFGR_SWS_PLL != (myRCC->CFGR & RCC_CFGR_SWS)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
	
	// Update SystemCoreClock variable
	SystemCoreClockUpdate();
	
	/* START - Set-up PD12, PD13 as low-speed gpio push-pull output */
	// Enable clock for GPIOD
	myRCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	// Set-up PD12 as slow speed general purpose push-pull output
	myPortD->MODER |= GPIO_MODER_MODE12_0;
	myPortD->MODER &= ~(GPIO_MODER_MODE12_1);
	myPortD->OTYPER &= ~(GPIO_OTYPER_OT12);
	myPortD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12);
	myPortD->PUPDR  &= ~(GPIO_PUPDR_PUPD12);
	
	// Set-up PD13 as slow speed general purpose push-pull output
	myPortD->MODER |= GPIO_MODER_MODE13_0;
	myPortD->MODER &= ~(GPIO_MODER_MODE13_1);
	myPortD->OTYPER &= ~(GPIO_OTYPER_OT13);
	myPortD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED13);
	myPortD->PUPDR  &= ~(GPIO_PUPDR_PUPD13);
	/* END - Set-up PD12, PD13 as low-speed push-pull gpo */

#ifdef SYSTICK_ENABLE
	// Set-up for a systick of every 1ms
	SysTick_Config((SystemCoreClock/1000) - 1);
#endif
	
	// TBR - Code to demo watchdog reset
	msec_Delay(2000);
	myPortD->BSRR |= GPIO_BSRR_BS13;
	
#if defined(SYSTICK_ENABLE) && defined(WATHDOG_ENABLE)
	// Set-up IWDG for reset of 32768ms
	myIWatchDog->KR = IWDG_UNLOCK_KEY;
	myIWatchDog->PR = IWDG_PRESCALER_256;
	myIWatchDog->RLR = 4095;
	myIWatchDog->KR = IWDG_START_KEY;
#endif
	
	// Enable printf thru ITM Port 0
	printf("Hello World!\n");
	
	/* START - Set-up PB6, PB7, PB8, PB9 as low-speed gpio push-pull output w/ pulldown; muxed to AF2 */
	//Enable clock for PORTB
	myRCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	// Set-up PB6, PB7, PB8, PB9 as slow speed general purpose push-pull output w/ pulldown
	myPortB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	myPortB->MODER |= (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0);
	myPortB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	myPortB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	myPortB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
//	myPortB->PUPDR |= GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1 | GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD9_1;
	myPortB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0;

//	// Set-up PB6:PB7:PB8:PB9 to alternate funtion 2 i.e. TIM4_CH1:2:3:4
//	myPortB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
//	myPortB->AFR[0] |= GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL7_1;
//	myPortB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
//	myPortB->AFR[1] |= GPIO_AFRH_AFSEL8_1 | GPIO_AFRH_AFSEL9_1;
//	/* END - Set-up PB6, PB7, PB8, PB9 as low-speed gpio push-pull output w/ pulldown; muxed to AF2 */
	
//	/* START - Set-up TIMER4 for edge-aligned PWM@10KHz with outputs on CH1:2:3:4 */
//	// Enable clock for TIMER4
//	myRCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
//	
//	// Enable ALL timer4 update events; counts-up; edge-aligned pwm
//	myTimer4->CR1 = TIMER4_CR1_RESET;
//	myTimer4->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_DIR | TIM_CR1_CMS);
//	myTimer4->CR1 |= TIM_CR1_ARPE;
//	
//	
//	// CH2:1 PWM MODE 1 outputs w/ preloads; disable fast-enable;
//	myTimer4->CCMR1 = TIMER4_CCMR1_RESET;
//	myTimer4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE );
//	myTimer4->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
//	myTimer4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
//	
//	// CH4:3 PWM MODE 1 outputs w/ preloads; disable fast-enable;
//	myTimer4->CCMR2 = TIMER4_CCMR2_RESET;
//	myTimer4->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3FE |TIM_CCMR2_CC4S | TIM_CCMR2_OC4FE);
//	myTimer4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
//	myTimer4->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
//	
//	//
//	myTimer4->CCER = TIMER4_CCER_RESET;
//	myTimer4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | 
//											TIM_CCER_CC2P | TIM_CCER_CC2NP |
//											TIM_CCER_CC3P | TIM_CCER_CC3NP |
//											TIM_CCER_CC4P | TIM_CCER_CC4NP);
//	myTimer4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
//	
//	// No prescaler
//	myTimer4->PSC = TIMER4_PSC_RESET;
//	
//	// PWM@10Khz w/ APB1@42Mhz
//	myTimer4->ARR = 4200;
//	
//	// CH1:2:3:4 @50% duty cycle
//	myTimer4->CCR1 = 2200;
//	myTimer4->CCR2 = 2200;
//	myTimer4->CCR3 = 2200;
//	myTimer4->CCR4 = 2200;
//	
//	// Enable timer4
//	myTimer4->CR1 |= TIM_CR1_CEN;
//	/* END - Set-up TIMER4 for edge-aligned PWM@10KHz with outputs on CH1:2:3:4 */
	
	
	while(TRUE){
		
		// Atomically set PD12
		myPortD->BSRR |= GPIO_BSRR_BS12;
		msec_Delay(1000);
		//for( tmpctr = DELAY_COUNTER; tmpctr > 0; --tmpctr);
		// Atomically reset PD12
		myPortD->BSRR |= GPIO_BSRR_BR12;
		//for( tmpctr = DELAY_COUNTER; tmpctr > 0; --tmpctr);
		msec_Delay(1000);
	}
	
	
	
	return 0;
}



void SysTick_Handler(void){
	
	// Re-load watchdog counter
#if defined(SYSTICK_ENABLE) && defined(WATHDOG_ENABLE)
	myIWatchDog->KR = 0xAAAA;
#endif
	
	if(Delay > 0) --Delay;
}



#if defined(SYSTICK_ENABLE)
void msec_Delay(uint32_t nTime){
	Delay = nTime;
	while(Delay != 0);
}

#else
void msec_Delay(uint32_t nTime){
	Delay = nTime * (SystemCoreClock/(FREQ_AN_MSEC * MSEC_CALIBRATION_FACTOR));
	while(Delay != 0) --Delay;
}
#endif



void Manage_Timeout(uint32_t nTimeout){
	while(nTimeout == TIMED_OUT);
}
