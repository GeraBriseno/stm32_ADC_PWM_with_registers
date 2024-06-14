#include "stm32f401xe.h"
#include <math.h>

// Variable to store output of potentiometer after being normalized to 0-100
static uint32_t output_pot = 0;

// Variable to store the slope used for normalization of potentiometer ADC value
static double slope = 1.0 * 100/4095;

// Function used to round value obtained from normalization of potentiometer ADC value
static uint32_t roundFunction(double d){
    return floor(d + 0.5);
}

//Variable to store value from ADC reading
static uint16_t ADC_VAL = 0;

/* Function used to configure System Clock, in this case we're using the external crystal oscillator in
 ST-Link debugger connected to Nucleo Board STM32F401RE using Phase Locked Loop (PLL) */
static void SysClockConfig (void){
	
	/* Frequency of crystal oscillator is 8MHz, to calculate frequency obtained with PLL, we use the formula:
	 ((inputfreq/PLL_M)*PLL_N)/PLL_P  in this case ((80/4)*80)/2 */
	
	/* Values to write to reset and clock control pll configuration register (RCC->PLLCFGR) to get desired clock values
	 (these values have been obtained using STM32CubeIDE) */
	#define PLL_M 	4
	#define PLL_N 	80
	#define PLL_P 	0  // PLLP = 2

	// 1. Enable HSE (high speed external clock source) and wait for the HSE to become ready
	RCC->CR |= RCC_CR_HSEON;  // RCC->CR |= 1<<16;
	while (!(RCC->CR & RCC_CR_HSERDY));  // while (!(RCC->CR & (1<<17)));
	
	// 2. Enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // RCC->APB1ENR |= 1<<28;
	// Set VOS (voltage output scaling) in PWR register as reserved (scale 2 mode selected) to allow a maximum freq of 84 MHz
	PWR->CR |= PWR_CR_VOS;  // PWR->CR |= 3<<14; 
	
	/* 3. Configure the flash prefetch and the latency related settings
	 (these values have been obtained using STM32CubeIDE) */
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;  // FLASH->ACR = (1<<8) | (1<<9)| (1<<10)| (5<<0);
	
	/* 4. Configure the prescalers to get final values for HCLK, PCLK1, PCLK2
	 (these values have been obtained using STM32CubeIDE) */
	
	// AHB prescaler = 1
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // RCC->CFGR |= (0<<4);
	
	// APB1 prescaler = 2
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // RCC->CFGR |= (4<<10);
	
	// APB2 prescaler = 1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  // RCC->CFGR |= (0<<13);
	
	// 5. Configure the main PLL
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P <<16) | (RCC_PLLCFGR_PLLSRC_HSE);  // (1<<22);

	// 6. Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;  // RCC->CR |= (1<<24);
	while (!(RCC->CR & RCC_CR_PLLRDY));  // while (!(RCC->CR & (1<<25)));
	
	// 7. Select the Clock Source as PLL and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // RCC->CFGR |= (2<<0);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // while (!(RCC->CFGR & (2<<2)));
	
}


//Function to configure GPIO port and pins used, in this case GPIO port A
static void GPIO_Config(void){
	
	// 1. Enable the GPIOA CLOCK
	RCC->AHB1ENR |= (1<<0);
	
	/* 2. Set the Respective GPIO PINs as Output Mode
	GPIOA->MODER |= (1<<12);  // pin PA6(bits 13:12) as Output (01) */
	
	// 2. Configure the OUTPUT MODE
	GPIOA->OTYPER = 0;
	GPIOA->OSPEEDR = 0;
	
	// 3. Set the respective GPIO pins as Alternate Mode
	GPIOA->MODER |= (2<<0);  // alternate mode for PA 0
	GPIOA->MODER |= (2<<2);  // alternate mode for PA 1
	GPIOA->MODER |= (2<<10);  // alternate mode for PA 5
	
	// 3. Set the respective GPIO pins' Alternate Function
	GPIOA->AFR[0] |= (2<<0);  //set alternate function as af2 (TIM5_CHANNEL_1)
	GPIOA->AFR[0] |= (1<<4);  //set alternate function as af1 (TIM2_CHANNEL_2)
	GPIOA->AFR[0] |= (1<<20);  //set alternate function as af1 (TIM2_CHANNEL_1)
	
	//5. Set the Respective GPIO PINs to Analog Mode	
	GPIOA->MODER |= (3<<8);  // analog mode for PA 4 (chennel 4)
	
}


//Function to configure Timer 2 for use in PWM output 
static void configureTIM2(void){
	
	//Enable TIM2 peripheral clock
	RCC->APB1ENR |= (1 << 0);
	
	//Channels 1 and 2 configured as output
	TIM2->CCMR1 |= (0<<0);
	TIM2->CCMR1 |= (0<<8);
	
	//Set capture/compare polarity as active low for channels 1 and 2
	TIM2->CCER &=~ TIM_CCER_CC1P;  // |= (1<<1)
	TIM2->CCER &=~ TIM_CCER_CC2P;  // |= (1<<5)
	
	//Set output compare mode as PWM mode 1 for channels 1 and 2
	TIM2->CCMR1 |= (6<<4);
	TIM2->CCMR1 |= (6<<12);
	
	//Set prescaler value for TIM2, to achieve a frequency of 1KHz
	TIM2->PSC = 79999;     //SET TIM2 FREQUENCY TO 1 KHz (80,000,000/(79,999+1)) USING PRESCALER
	
	//Set autoreload value for TIM2, this is the period for our PWM output
	TIM2->ARR = (uint32_t)100;   //PERIOD   //10HZ
	
	//Set value for capture compare register for channels 1 and 2, this will be the duty of our PWM output
	TIM2->CCR1 = 100;    //DUTY
	TIM2->CCR2 = 100;    //DUTY
	
	//Enable output compare preload for channels 1 and 2
	TIM2->CCMR1 |= (1<<3);
	TIM2->CCMR1 |= (1<<11);
	
	//Enable autoreload preload for TIM2
	TIM2->CR1 |= (1<<7);
	
	//Select edge aligned mode for TIM2, the counter counts up or down depending on the direction bit
	TIM2->CR1 |= (0<<5);
	
	//Set direction bit as upcounter
	TIM2->CR1 |= (0<<4);
	
	//Enable capture/compare signal as output in corresponding channel pins
	TIM2->CCER |= (1<<0);
	TIM2->CCER |= (1<<4);
	
	//Enable TIM2 counter
	TIM2->CR1 |= (1<<0);
}


//Function to configure Timer 5 for use in PWM output 
static void configureTIM5(void){
	
	//Enable TIM5 peripheral clock
	RCC->APB1ENR |= (1 << 3);
	
	//Channel 1 is configured as output
	TIM5->CCMR1 |= (0<<0);
	
	//Set capture/compare polarity as active low for channel 1
	TIM5->CCER &=~ TIM_CCER_CC1P;  // |= (1<<1)
	
	//Set output compare mode as PWM mode 1 for channel 1
	TIM5->CCMR1 |= (6<<4);
	
	//Set prescaler value for TIM5, to achieve a frequency of 1KHz
	TIM5->PSC = 79999;     //SET TIM5 FREQUENCY TO 1 KHz (80,000,000/(79,999+1)) USING PRESCALER
	
	//Set autoreload value for TIM5, this is the period for our PWM output
	TIM5->ARR = (uint32_t)100;   //PERIOD  //10HZ
	
	//Set value for capture compare register for channels 1 and 2, this will be the duty of our PWM output
	TIM5->CCR1 = 100;    //DUTY
	
	//Enable output compare preload for channel 1
	TIM5->CCMR1 |= (1<<3);
	
	//Enable autoreload preload for TIM5
	TIM5->CR1 |= (1<<7);
	
	//Select edge aligned mode for TIM5, the counter counts up or down depending on the direction bit
	TIM5->CR1 |= (0<<5);
	
	//Set direction bit as upcounter
	TIM5->CR1 |= (0<<4);
	
	//Enable capture/compare signal as output in corresponding channel pin
	TIM5->CCER |= (1<<0);
	
	//Enable TIM5 counter
	TIM5->CR1 |= (1<<0);
}


//Function to initialize ADC
static void ADC_Init(){
	
	//1. Enable ADC1 clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	
	//2. Set the prescaler in the ADC Common Control Register
	ADC->CCR |= 1<<16;  		 // PCLK2 divided by 4 (80/4)
		
	//3. Set the scan mode and resolution in the Control Register 1
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 |= (0<<24);   // 12 bit resolution
		
	//4. Set the continuous conversion, EOC (end of conversion), and data alignment in Control Register 2 (CR2)
	ADC1->CR2 |= (1<<1);     // Enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOCS after each conversion
	ADC1->CR2 |= (0<<11);   // Data alignment right
		
	//5. Set the sampling time for the channels	
	ADC1->SMPR2 |= (0<<12);  // Sampling time of 3 cycles for channel 4

	//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (0<<20);   // SQR1_L = 0 for 1 conversion, that is one channel
		
}


//Function to turn ADC on
static void ADC_Enable (void){
	
	//Turn ADC converter on
	ADC1->CR2 |= 1<<0;
	
	//Wait for ADC to stabilize (approx 10us)
	uint32_t delay = 10000;
	while (delay--);
	
}


// Function to start ADC conversion
static void ADC_Start (void){

	//Clear conversion sequence
	ADC1->SQR3 = 0;
	
	//Set channel in the conversion sequence, in this case we're only using channel 4, otherwise we can specify the channel
	ADC1->SQR3 |= (4<<0);
	
	//Clear status register
	ADC1->SR = 0;
	
	//Start conversion, by setting SWSTART bit in CR2
	ADC1->CR2 |= (1<<30);
}


// Function to wait for ADC conversion
static void ADC_WaitForConv (void){
	
	//Wait for EOC (end of conversion) flag to be set
	while (!(ADC1->SR & (1<<1)));
	
}


// Function to get ADC value from ADC1 data register
static uint16_t ADC_GetVal (void){
	
	// Read the data register
	return ADC1->DR;
}

// Our main function
int main (void){
	
	// Run our configuration functions
	SysClockConfig ();
	configureTIM2();
	configureTIM5();
	GPIO_Config();
	ADC_Init ();
	ADC_Enable ();	
	
	// Our while loop
	while (1)
	{	
		// Read value from ADC
		ADC_Start();
		ADC_WaitForConv();
		ADC_VAL = ADC_GetVal();
		
		//Normalize ADC input to a 0-100 scale
		output_pot = roundFunction(slope * ADC_VAL);
		
		//We're using the value read from ADC to modify the duty of our PWM outputs and change the brightness of 3 LEDs
		
		if(output_pot < 33){
			//Set duty of PWM to the normalized value of the ADC reading
			TIM5->CCR1 = output_pot;
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
		}
		else if((output_pot < 66) && (output_pot >= 33)){
			TIM5->CCR1 = output_pot;
			TIM2->CCR1 = output_pot;
			TIM2->CCR2 = 0;
		}
		else{
			TIM5->CCR1 = output_pot;
			TIM2->CCR1 = output_pot;
			TIM2->CCR2 = output_pot;
		}
				
	}
}
