#include "main.h"

#define dbl 100
#define ddl 28

static volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

volatile bool flag_ADCDMA_TransferComplete;

const uint16_t dd[] = {50,100,150,200,250,300,350,400,450,500,550,600,650,700,650,600,550,500,450,400,350,300,250,200,150,100,50,0};	
uint16_t db[dbl];

volatile uint16_t systick_ms = 0, toggle_ms = 0;

uint8_t i;

volatile uint8_t flag_UserButton = 0;

void SysTick_Handler(void) {
    TimingDelay_Decrement();
}

void EXTI0_IRQHandler(void)
{
  /* set UserButton Flag */
	flag_UserButton = 1;
  EXTI_ClearITPendingBit(EXTI_Line0);
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 		GPIO_TOGGLE(GPIOA,	GPIO_Pin_9);
		
  }

}

void DMA1_Channel1_IRQHandler    (void)
{
  
	if (DMA_GetFlagStatus(DMA1_IT_TC1)!= RESET)
	{
		DMA_ClearFlag(DMA1_IT_TC1);
		//DMA_ClearFlag(DMA_IT_TC);
		setADCDMA_TransferComplete();  /* set flag_ADCDMA_TransferComplete global flag */
		
		GPIO_TOGGLE(GPIOA,	GPIO_Pin_8);
	}
}



int main(void){
	int ch_index;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
 
	RCC_Configuration();

	led_init();
	
	pin_mode(GPIOA, GPIO_Pin_9, GPIO_MODE_OUT2_PP);
	GPIO_HIGH(GPIOA,	GPIO_Pin_9);
	pin_mode(GPIOA, GPIO_Pin_8, GPIO_MODE_OUT2_PP);
	GPIO_HIGH(GPIOA,	GPIO_Pin_8);
	pin_mode(GPIOA, GPIO_Pin_10, GPIO_MODE_OUT2_PP);
	GPIO_HIGH(GPIOA,	GPIO_Pin_10);
	
	DAC_DMA_Config();

	
	pin_mode(GPIOC, GPIO_Pin_3, GPIO_MODE_IN_AN);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);		
	
	SetCalibData();
	
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div1;
  ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	          			
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			  							
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  
  ADC_InitStructure.ADC_NbrOfConversion = 1;             
  ADC_Init(ADC1, &ADC_InitStructure);
	
	for (ch_index = 1; ch_index <= dbl; ch_index++){
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, ch_index, 
														 ADC_SampleTime_384Cycles);
	}
	
  ADC_Cmd(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET); 

  DMA_DeInit(DMA1_Channel1);
  
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     		 
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&db;  
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         
  DMA_InitStructure.DMA_BufferSize = dbl;                     
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     		 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     	  
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     	 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);		

  DMA_ITConfig(DMA1_Channel1, DMA1_IT_TC1, ENABLE);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_DMACmd(ADC1, ENABLE);

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	

	while(1)
	{
	
		//GPIO_TOGGLE(GPIOA,	GPIO_Pin_10);		

		ADC_DMACmd(ADC1, DISABLE);
		ADC_DMACmd(ADC1, ENABLE);
	  clearADCDMA_TransferComplete();
		ADC_SoftwareStartConv(ADC1);
		while (!flag_ADCDMA_TransferComplete);			
		
 		if (uint16_time_diff(systick_ms, toggle_ms) >= 1000) //1 sec delay
 		{
 			toggle_ms = systick_ms;

			GPIO_TOGGLE(LD_PORT,LD_BLUE);
				
		}
		
	}
}


void DAC_DMA_Config(void){
	DAC_InitTypeDef DAC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_DAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	pin_mode(DAC_GPIO, DAC_OUT1, GPIO_MODE_IN_AN);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	//TIM_TimeBaseStructure.TIM_Period = 0x300;
	TIM_TimeBaseStructure.TIM_Period = 0x150;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & DAC->DHR12R1;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &dd;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = ddl;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel2, ENABLE);

	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = 0;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO; //DAC triger on TIM2
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);

	DAC_DMACmd(DAC_Channel_1, ENABLE);

	//dac_set(DAC_Channel_1, 600);

	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE); //DMA update on TIM2

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //timer test on PA09
	
	TIM_Cmd(TIM2, ENABLE);	

	NVIC_EnableIRQ(TIM2_IRQn);	//IRQ for timer test
	
}

void RCC_Configuration(void){

  //SysTick_Config(SystemCoreClock / 4000);
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000); //for msec

  /* Enable the GPIOs Clock */
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);     
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		
}  




uint16_t uint16_time_diff(uint16_t now, uint16_t before)
{
  return (now >= before) ? (now - before) : (UINT16_MAX - before + now);
}

void Delay(uint32_t nTime){
  TimingDelay = nTime;

  while(TimingDelay != 0);
  
}

void TimingDelay_Decrement(void){

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
	
	++systick_ms;
}

void setADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = TRUE;
}

void clearADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = FALSE;
}


