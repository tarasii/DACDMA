/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

/* Includes ------------------------------------------------------------------*/
#include "utils.h"

/* Exported constants --------------------------------------------------------*/

#define MAX_ADC_CHNL 12

#define MAX_TEMP_CHNL 4

#define ADC_CONV_BUFF_SIZE 20

#define FACTORY_CALIB_BASE        ((uint32_t)0x1FF80078)    /*!< Calibration Data Bytes base address */
#define FACTORY_CALIB_DATA        ((CALIB_TypeDef *) FACTORY_CALIB_BASE)
#define USER_CALIB_BASE           ((uint32_t)0x08080000)    /*!< USER Calibration Data Bytes base address */
#define USER_CALIB_DATA           ((CALIB_TypeDef *) USER_CALIB_BASE)
#define TEST_CALIB_DIFF           (int32_t) 50  /* difference of hot-cold calib
                                               data to be considered as valid */ 

#define HOT_CAL_TEMP 110
#define COLD_CAL_TEMP  25

#define DEFAULT_HOT_VAL 0x362
#define DEFAULT_COLD_VAL 0x2A8

/* Theorically BandGAP 1.224volt */
#define VREF 		1.224L
/*
	ADC Converter 
	LSBIdeal = VREF/4096 or VDA/4096
*/
#define ADC_CONV 	4096

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t myVREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;

typedef struct {
	uint32_t refAVG; 
	uint32_t tempAVG;
	int32_t temperature_C;
	float voltage_V;	
	uint32_t Chanel13AVG; 
} ADC_Typedef;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void acquireTemperatureData(void);
void insertionSort(uint16_t *numbers, uint32_t array_size); 
uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples);
void processTempData(ADC_Typedef* ADC_RES);
void adc_init(void);
void configureDMA(void);

uint16_t adc_coretemp_simple(void);

#endif /* __ARC_H */
