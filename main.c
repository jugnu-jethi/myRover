#include "main.h"



int main(){
	
	GPIO_TypeDef *myPortA = GPIOA;
	GPIO_TypeDef *myPortB = GPIOB;
	FLASH_TypeDef *myFlash = FLASH;
	SYSCFG_TypeDef *mySysCfg = SYSCFG;
	PWR_TypeDef *myPowerInterface = PWR;
	DBGMCU_TypeDef *myMCUDebug = DBGMCU;
	
	
	
#if defined(WATCHDOG_ENABLE)
	// Set-up IWDG for reset of 32768ms
	myIWatchDog->KR = IWDG_UNLOCK_KEY;
	myIWatchDog->PR = IWDG_PRESCALER_256;
	myIWatchDog->RLR = IWDG_RELOAD_VALUE;
	myIWatchDog->KR = IWDG_START_KEY;
#endif
	
	/* START - Set-up PD12, PD13, PD14, PD15 as low-speed gpio push-pull output */
	// Enable clock for GPIOD
	myRCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	// Set-up PD12:13:14:15 as slow speed general purpose push-pull output
	myPortD->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	myPortD->MODER &= ~(GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1);
	myPortD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15);
	myPortD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);
	myPortD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15);
	/* END - Set-up PD12, PD13, PD14, PD15 as low-speed gpio push-pull output */
	
#if defined(WATCHDOG_ENABLE)
	// Indicate IWDG reset occured by setting amber LED@PD13
	if(RCC_CSR_WDGRSTF == (myRCC->CSR & RCC_CSR_WDGRSTF)){
		myPortD->BSRR |= GPIO_BSRR_BS13;
	}
#endif
	
	/* START - Configure Device Clock to Max Frequency @168Mhz using External Crystal @8Mhz */
	// Enable clock to Power Interface
	myRCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// Scale 1.2V-PSU to Scale 1
	myPowerInterface->CR |= PWR_CR_VOS;
	
	// Set flash wait states to 5
	// Reference Manual: Section 3.5.1
	myFlash->ACR &= ~(FLASH_ACR_LATENCY);
	myFlash->ACR |= FLASH_ACR_LATENCY_5WS;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(FLASH_ACR_LATENCY_5WS != (myFlash->ACR & FLASH_ACR_LATENCY)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
	// Enable prefetch if Rev Z device; Enable Instruction Cache; Enable Data Cache
	myFlash->ACR |= (REVISION_Z == (myMCUDebug->IDCODE & DBGMCU_IDCODE_REV_ID) ? FLASH_ACR_PRFTEN : 0U) | 
									FLASH_ACR_ICEN | 
									FLASH_ACR_DCEN;
	
	// Enable HSE oscillator source i.e. 8Mhz crystal
	myRCC->CR &= ~(RCC_CR_HSEBYP);
	myRCC->CR |= RCC_CR_HSEON;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(RCC_CR_HSERDY != (myRCC->CR & RCC_CR_HSERDY)){
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
	while(RCC_CR_PLLRDY != (myRCC->CR & RCC_CR_PLLRDY)){
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
	/* END - Configure Device Clock to Max Frequency @168Mhz using External Crystal @8Mhz */
	
#if defined(IO_COMPENSATION_CELL_ENABLE)
	// Enable clock for SYSCFG
	myRCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	mySysCfg->CMPCR |= SYSCFG_CMPCR_CMP_PD;
	timeout = CONFIG_TIMEOUT_DURATION;
	while(SYSCFG_CMPCR_READY != (mySysCfg->CMPCR & SYSCFG_CMPCR_READY)){
		if(timeout != TIMED_OUT) --timeout;
	}
	Manage_Timeout(timeout);
#endif
	

	//warm-up print; otherwise characters go missing in subsequent calls
	printf("***********************************************************\n"); 
	
	printf("Revision ID: %#x\n", (myMCUDebug->IDCODE & DBGMCU_IDCODE_REV_ID));

#if defined(RTE_Compiler_IO_STDOUT_ITM )
	// Enable printf thru ITM Port 0
	printf("Hello World!\n");
#endif
	
	/* START - Set-up PB6, PB7, PB8, PB9 as low-speed gpio push-pull output w/ pulldown; muxed to AF2 */
	//Enable clock for PORTB
	myRCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	// Set-up PB6, PB7, PB8, PB9 as slow speed ALTERNATE purpose push-pull output w/ pulldown
	myPortB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	myPortB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
	myPortB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	myPortB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	//myPortB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9); // very hi-speed
	//myPortB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR9_0; // medium speed
	myPortB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
	myPortB->PUPDR |= GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1 | GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD9_1;
	//myPortB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0; // pull-up

	// Set-up PB6:PB7:PB8:PB9 to alternate funtion 2 i.e. TIM4_CH1:2:3:4
	myPortB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	myPortB->AFR[0] |= GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL7_1;
	myPortB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	myPortB->AFR[1] |= GPIO_AFRH_AFSEL8_1 | GPIO_AFRH_AFSEL9_1;
	/* END - Set-up PB6, PB7, PB8, PB9 as low-speed gpio push-pull output w/ pulldown; muxed to AF2 */
	
	/* START - Set-up TIMER4 for edge-aligned PWM@10KHz with outputs on CH1:2:3:4 */
	// Enable clock for TIMER4
	myRCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// Enable ALL timer4 update events; counts-up; edge-aligned pwm
	myTimer4->CR1 = TIMER4_CR1_RESET;
	myTimer4->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_DIR | TIM_CR1_CMS);
	myTimer4->CR1 |= TIM_CR1_ARPE;
	
	
	// CH1:2 PWM MODE 1 outputs w/ preloads; disable fast-enable;
	myTimer4->CCMR1 = TIMER4_CCMR1_RESET;
	myTimer4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE );
	myTimer4->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	myTimer4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	
	// CH3:4 PWM MODE 1 outputs w/ preloads; disable fast-enable;
	myTimer4->CCMR2 = TIMER4_CCMR2_RESET;
	myTimer4->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3FE |TIM_CCMR2_CC4S | TIM_CCMR2_OC4FE);
	myTimer4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	myTimer4->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	
	//
	myTimer4->CCER = TIMER4_CCER_RESET;
	myTimer4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | 
											TIM_CCER_CC2P | TIM_CCER_CC2NP |
											TIM_CCER_CC3P | TIM_CCER_CC3NP |
											TIM_CCER_CC4P | TIM_CCER_CC4NP);
	myTimer4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	
	// No prescaler
	myTimer4->PSC = TIMER4_PSC_VALUE;
	
	// PWM@10Khz w/ APB1@42Mhz
	// Note: Timer4 RCC input clock is doubled internally to 84Mhz(42Mhz x 2)
	myTimer4->ARR = TIMER4_PWM_PERIOD;
	
	// CH1:2:3:4 @50% duty cycle
	myTimer4->CCR1 = 2100;
	myTimer4->CCR2 = 2100;
	myTimer4->CCR3 = 2100;
	myTimer4->CCR4 = 2100;
	
	// Enable timer4
	myTimer4->CR1 |= TIM_CR1_CEN;
	/* END - Set-up TIMER4 for edge-aligned PWM@10KHz with outputs on CH1:2:3:4 */
	
	/* START - Set-up push button on PA0; Attach to EXTI0 */
	// Enable clock for GPIOA
	myRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// Set-up PA0 as slow speed general purpose push-pull input
	myPortA->MODER &= ~(GPIO_MODER_MODE0);
	myPortA->OTYPER &= ~(GPIO_OTYPER_OT0);
	myPortA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0);
	myPortA->PUPDR &= ~(GPIO_PUPDR_PUPD0);

	// Select EXTI0 interrupt on pin PA0
	mySysCfg->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0);
	mySysCfg->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0 & SYSCFG_EXTICR1_EXTI0_PA);
	
	// Enable EXTI0 interrupt w/ rising trigger edge
	myEXTI->IMR |= EXTI_IMR_MR0;
	myEXTI->RTSR |= EXTI_RTSR_TR0;
	
	__NVIC_EnableIRQ(EXTI0_IRQn);
	/* END - Set-up push button on PA0 */
	
	/* START - Configure ADC1 to read from channel ADC1_IN1@PA1(single-conversion) */
	// Enable clock for GPIOA
	/* Not needed as already enabled prior
	//myRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; */
	
	// Set-up PA1 as slow speed general purpose push-pull analog input
	myPortA->MODER |= GPIO_MODER_MODE1;
	myPortA->OTYPER &= ~(GPIO_OTYPER_OT1);
	myPortA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED1);
	myPortA->PUPDR &= ~(GPIO_PUPDR_PUPD1);
	
	// Enable clock for ADC1 
	myRCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	//Set-up clock for ALL ADCs, ADCCLK, to PCLK2/4
	ADCCommonControl->CCR &= ADC_CCR_ADCPRE;
	ADCCommonControl->CCR |= ADC_CCR_ADCPRE_0;
	
	// 12-bit resolution;
	myADC1->CR1 &= ~(ADC1_CR1_RESET);
	myADC1->CR1 &= ~(ADC_CR1_RES);
	
	// right align; EOC flag enabled; single-conversion mode
	myADC1->CR2 &= ~(ADC1_CR2_RESET);
	myADC1->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_EOCS);
	
	// sampling time for channel ADC1_IN1 of 3 cycles
	myADC1->SMPR2 &= ~(ADC_SMPR2_SMP1);
	
	// First group sequence is channel 1 i.e. ADC1_IN1
	myADC1->SQR3 &= ~(ADC_SQR3_SQ1);
	myADC1->SQR3 |= ADC_SQR3_SQ1_0;
	
	// Total number of conversion is ONE
	myADC1->SQR1 &= ~(ADC_SQR1_L);
	
	// Switch ADC1 ON
	myADC1->CR2 |= ADC_CR2_ADON;
	/* END - Configure ADC1 to read from PA1 */
	
	
	
	/* START - Create FreeRTOS Tasks */
	if(pdFAIL == xTaskCreate(LEDHeartBeat, "HeartBeat", 50, NULL, 1, NULL)){
		
		printf("LED Heart Beat Task Failed!\n");
	}
	
#if defined(WATCHDOG_ENABLE)
	if(pdFAIL == xTaskCreate(IWDGCounterReload, "IWDGReset", 50, NULL, 1, NULL)){
		
		printf("IWDG Counter Reload Task Failed!\n");
	}
#endif
//	if(pdFAIL == xTaskCreate(TestPWM, "PWMTest", 50, NULL, 1, NULL)){
//		
//		printf("PWM Test Task Failed!\n");
//	}
	
	if(pdFAIL == xTaskCreate(SamplePOT, "SamplePOT", 50, NULL, 2, NULL)){
		
		printf("Sample POT Task Failed!\n");
	}
	
	if(pdFAIL == xTaskCreate(AdjustMotorSpeed, "ADJ-Motor", 50, NULL, 1, NULL)){
		
		printf("Adjust Motor Speed Task Failed!\n");
	}
	/* END - Create FreeRTOS Tasks */
	
	
	/* Start the scheduler so the tasks start executing. */
	vTaskStartScheduler();
	
	/* If all is well then main() will never reach here as the scheduler will
	now be running the tasks. If main() does reach here then it is likely that
	there was insufficient heap memory available for the idle task to be created.
	Chapter 2 provides more information on heap memory management. */
	while(TRUE){};
	
	return 0;
}



void EXTI0_IRQHandler(void){
	
	if(EXTI_PR_PR0 == (myEXTI->PR & EXTI_PR_PR0)) myEXTI->PR |= EXTI_PR_PR0;
	if(__NVIC_GetPendingIRQ(EXTI0_IRQn)) __NVIC_ClearPendingIRQ(EXTI0_IRQn);
	
	myTaskFlags |= 0x0001;
}



void msec_Delay(uint32_t nTime){
	
	Delay = nTime * (SystemCoreClock/(FREQ_AN_MSEC * MSEC_CALIBRATION_FACTOR));
	while(Delay != 0) --Delay;
}



void Manage_Timeout(uint32_t nTimeout){
	
	if(nTimeout == TIMED_OUT){
		
		myPortD->BSRR |= GPIO_BSRR_BS15;
		while(TRUE);
	}
}



void LEDHeartBeat(void *pvLED_HeartBeat){
	
	const TickType_t xDelay_1000ms = pdMS_TO_TICKS(1000);
	
	while(TRUE){
		
		// Atomically set Green-LED@PD12
		myPortD->BSRR |= GPIO_BSRR_BS12;
		vTaskDelay(xDelay_1000ms);
		// Atomically re-set Green-LED@PD12
		myPortD->BSRR |= GPIO_BSRR_BR12;
		vTaskDelay(xDelay_1000ms);
	}
	
	vTaskDelete( NULL );
}



#if defined(WATCHDOG_ENABLE)
void IWDGCounterReload(void *pvIWDG_Counter_Reload){
	
	const TickType_t xDelay_8000ms = pdMS_TO_TICKS(8000);
	
	while(TRUE){
		
#if defined(WATCHDOG_ENABLE) && defined(WATCHDOG_TEST_DISABLE)
		// Re-load watchdog counter
		myIWatchDog->KR = IWDG_RELOAD_KEY;
#endif
		
		// Re-set watchdog flag & Amber-LED@PD13
		if(RCC_CSR_WDGRSTF == (myRCC->CSR & RCC_CSR_WDGRSTF)){
			
			vTaskDelay(xDelay_8000ms);
			myPortD->BSRR |= GPIO_BSRR_BR13;
			myRCC->CSR |= RCC_CSR_RMVF;
			
			timeout = CONFIG_TIMEOUT_DURATION;
			while(RCC_CSR_WDGRSTF == (myRCC->CSR & RCC_CSR_WDGRSTF)){
				
				if(timeout != TIMED_OUT) --timeout;
			}
			Manage_Timeout(timeout);
		}
		
		vTaskDelay(xDelay_8000ms);
	}
	
	vTaskDelete( NULL );
}
#endif



void TestPWM(void *pvTest_PWM){
	
	const TickType_t xDelay_1000ms = pdMS_TO_TICKS(1000);
	
	while(TRUE){
		
		// Disable CH2@PB7
		myTimer4->CCER &= ~(TIM_CCER_CC2E);
		// Decrement duty cycle in 10% decrements@PB7
		if( 0 >= myTimer4->CCR2) myTimer4->CCR2 = TIMER4_PWM_PERIOD;
		else myTimer4->CCR2 -= 420;
		vTaskDelay(xDelay_1000ms);
		// Enable CH2@PB7
		myTimer4->CCER |= (TIM_CCER_CC2E);
		vTaskDelay(xDelay_1000ms);
	}
	
	vTaskDelete( NULL );
}



void SamplePOT(void *pvSample_POT){
	
	TickType_t xLastWakeTime;
	const TickType_t xSamplePeriod = pdMS_TO_TICKS( 100 );
	
	xLastWakeTime = xTaskGetTickCount();
	while(TRUE){
		
		vTaskDelayUntil( &xLastWakeTime, xSamplePeriod );
		
		// Start single-conversion on ADC1_IN1@PA1
		myADC1->CR2 |= ADC_CR2_SWSTART;
		
		// Wait for ADC1_IN1@PA1 results
		while(ADC_SR_EOC != (myADC1->SR & ADC_SR_EOC));
		if(ADC_SR_EOC == (myADC1->SR & ADC_SR_EOC)){
			
			// Calculate moving average to soften kinks
			PotsOhms = (PotsOhms + ADC1->DR)/2;
			
			//printf("ADC1_IN1@PA1: %d\n", PotsOhms);
		}
	}
	
	vTaskDelete( NULL );
}



void AdjustMotorSpeed( void * pvAdjust_Motor_Speed){
	
	const TickType_t xDelay_Adjust = pdMS_TO_TICKS(250);
	
	while(TRUE){
		
		vTaskDelay(xDelay_Adjust);
		myTimer4->CCR2 = PotsOhms;
		//printf("CH2@Duty-Cycle: %d\n", PotsOhms);
	}
	
	vTaskDelete( NULL );
}
