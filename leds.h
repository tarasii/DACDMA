/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEDS_H
#define __LEDS_H

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "pin.h"

/* Exported constants --------------------------------------------------------*/
#define LD_PORT 		GPIOB
#define LD_GREEN 		GPIO_Pin_7
#define LD_BLUE     GPIO_Pin_6

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void led_init(void);

#endif /* __LEDS_H */
