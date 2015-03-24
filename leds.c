#include "leds.h"

void led_init(void){

//  GPIO_InitTypeDef GPIO_InitStructure; 
//	
//	if (LD_PORT == GPIOA) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);     
//	if (LD_PORT == GPIOB) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);     
//	if (LD_PORT == GPIOC) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);     

//  /* Configure the GPIO_LED pins  LD3 & LD4*/
//  GPIO_InitStructure.GPIO_Pin   = LD_GREEN|LD_BLUE;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(LD_PORT, &GPIO_InitStructure);
	
	pin_mode(LD_PORT, LD_GREEN|LD_BLUE, GPIO_MODE_OUT2_PP);
  GPIO_LOW(LD_PORT, LD_GREEN);	
  GPIO_LOW(LD_PORT, LD_BLUE);
	
}

//void led_example(void){
//  GPIO_LOW(LD_PORT, LD_GREEN); //turn green off	
//  GPIO_HIGH(LD_PORT, LD_BLUE); //turn blue onn
//}
