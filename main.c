#include "main.h"

static volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

volatile bool flag_ADCDMA_TransferComplete;

const uint32_t dd[] = {50,100,150,200,250,300,350,400,450,500,550,600,650,700,650,600,550,500,450,400,350,300,250,200,150,100,50,0};
	int timerValue;

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
  DMA_ClearFlag(DMA1_IT_TC1);
  setADCDMA_TransferComplete();  /* set flag_ADCDMA_TransferComplete global flag */
}



int main(void){

	GPIO_InitTypeDef GPIO_InitStructure; 
	DAC_InitTypeDef DAC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 
	RCC_Configuration();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_DAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	led_init();
	
	pin_mode(GPIOA, GPIO_Pin_9, GPIO_MODE_OUT2_PP);
	GPIO_HIGH(GPIOA,	GPIO_Pin_9);
	
	GPIO_InitStructure.GPIO_Pin =  DAC_OUT1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DAC_GPIO, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0x300;
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
	DMA_InitStructure.DMA_BufferSize = 28;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
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

	while(1)
	{
	
		//GPIO_TOGGLE(GPIOA,	GPIO_Pin_9);		
		
 		if (uint16_time_diff(systick_ms, toggle_ms) >= 1000) //1 sec delay
 		{
 			toggle_ms = systick_ms;

			GPIO_TOGGLE(LD_PORT,LD_BLUE);
				
		}
		
	}
}




void RCC_Configuration(void){

  //SysTick_Config(SystemCoreClock / 4000);
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000);

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


