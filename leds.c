#include "leds.h"

void led_init(void){

	pin_mode(LD_PORT, LD_GREEN|LD_BLUE, GPIO_MODE_OUT2_PP);
  GPIO_LOW(LD_PORT, LD_GREEN);	
  GPIO_LOW(LD_PORT, LD_BLUE);
	
}

//void led_example(void){
//  GPIO_LOW(LD_PORT, LD_GREEN); //turn green off	
//  GPIO_HIGH(LD_PORT, LD_BLUE); //turn blue onn
//}
