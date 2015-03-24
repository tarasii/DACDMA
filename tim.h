/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

/* Includes ------------------------------------------------------------------*/
#include "utils.h"

/* Exported constants --------------------------------------------------------*/
//+------+------+------+------+
//| TIM2                      |
//+------+------+------+------+
//| CH1  | CH2  | CH3  | CH4  |
//+------+------+------+------+
//| PA0  | PA1  | PA2  | PA3  |
//| PA5  |      |      |      |
//| PA15 | PB3  | PB10 | PB11 |
//+------+------+------+------+
//| TIM3                      |
//+------+------+------+------+
//| PA6  | PA7  | PB0  | PB1  |
//| PB4  | PB5  |      |      |
//| PC6  | PC7  | PC8  | PC9  |
//+------+------+------+------+
//| TIM4                      |
//+------+------+------+------+
//| PB6  | PB7  | PB8  | PB9  |
//+------+------+------+------+
//#define TIM2_CH1 		GPIO_Pin_0
#define TIM2_CH1 		GPIO_Pin_5
#define TIM2_GPIO 	GPIOA
#define TIM4_CH3 		GPIO_Pin_8
#define TIM4_GPIO 	GPIOB


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void tim_init_cnt(void);
void tim_init_pwmout(uint32_t period, uint32_t dirty_cucle);

#endif /* __TIM_H */
