/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILS_H
#define __UTILS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>
//#include <misc.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define bool _Bool
#define FALSE 0
#define TRUE !FALSE

/* Exported macro ------------------------------------------------------------*/
/* MACROs for SET, RESET or TOGGLE Output port */

/* Exported functions ------------------------------------------------------- */
void Delay(uint32_t nTime);

#endif /* __UTILS_H */
