/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "pin.h"

/* Exported constants --------------------------------------------------------*/
#define BUTTON_GPIO_PORT	GPIOA
#define USER_GPIO_PIN		GPIO_Pin_0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void button_init_irq (void);
void button_init (void);

#endif /* __BUTTON_H */

