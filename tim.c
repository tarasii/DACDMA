#include "tim.h"


void tim_init_cnt(void){
	TIM_TimeBaseInitTypeDef TIM_InitBaseStructure;
 	TIM_ICInitTypeDef TIM_ICInitStruct;
  //NVIC_InitTypeDef NVIC_InitStructure; //create NVIC structure
  GPIO_InitTypeDef GPIO_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	/* TIM inputs */
	GPIO_InitStructure.GPIO_Pin = TIM2_CH1;                               
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(TIM2_GPIO, &GPIO_InitStructure);

  GPIO_PinAFConfig(TIM2_GPIO, GPIO_PinSource5, GPIO_AF_TIM2);
	
	TIM_TimeBaseStructInit(&TIM_InitBaseStructure);
//	TIM_InitBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM2, &TIM_InitBaseStructure);
	
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
  //TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
 	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling,
	//TIM_ICInit(TIM2, &TIM_ICInitStruct);
	TIM_PWMIConfig(TIM2, &TIM_ICInitStruct); 

	//TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	
//   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM2, ENABLE);
	
	NVIC_EnableIRQ(TIM2_IRQn);	
}

void tim_init_pwmout(uint32_t period, uint32_t dirty_cucle){

	TIM_TimeBaseInitTypeDef TIM_InitBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_OCInitTypeDef TIM_OCInit;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);	
	
 	GPIO_InitStructure.GPIO_Pin = TIM4_CH3;                               
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(TIM4_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(TIM4_GPIO, GPIO_PinSource8, GPIO_AF_TIM4);
	
	TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInit.TIM_Pulse = dirty_cucle;
	TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM4, &TIM_OCInit);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_TimeBaseStructInit(&TIM_InitBaseStructure);
	TIM_InitBaseStructure.TIM_Prescaler = 0;
	TIM_InitBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitBaseStructure.TIM_Period = period;
	TIM_InitBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_InitBaseStructure);
	
	TIM_Cmd(TIM4, ENABLE);
	
}


