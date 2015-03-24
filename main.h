/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "uart.h"
#include "leds.h"
#include "adc.h"
#include "tim.h"
#include "button.h"
#include "dac.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void TimingDelay_Decrement(void);
void RCC_Configuration(void);
uint16_t uint16_time_diff(uint16_t now, uint16_t before);
void setADCDMA_TransferComplete(void);
void clearADCDMA_TransferComplete(void);

#endif /* __MAIN_H */
