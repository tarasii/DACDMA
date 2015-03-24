BMP085
======

Simple Keil 4 project for STM32L-DISCOVERY board with DHT22, DS18B20 and BMP085 sensor connected.

Preasure, temperature and humidity are transmited to USART. Also timer pwm generation and pwm capture mode, ADC and DAC, RTC examle.

USART config: 9600,8,N,1

To get board responce, we need to send any char to USART 


pins:


RS232 (USART2)
PA10:  TxD
PA09:  RxD

DHT22
PC02:  OUT

BMP084 (I2C2)
PB11:  SDA
PB10:  SLC

DS18B20 (USART1)
PA02:  OUT
PA03:  OUT

TIM2 (in) TIM4 (out)
PA05: PA15

ADC (in) DAC (out)
PA04: PC03

Responce example:
UART=103;
MODE=3;
V_ref_RAW=1689;
Vref=2.97V;
T_core_RAW=677;
Tcore=24C;
DHT_RAW=012b01022f;
T_DHT=25.8C;
H_DHT=29.9%;
BMP085_Calib 1e35 fbca c86d 836a 6223 6175 157a 0038 8000 d4bd 0980;
BMP085_RAW 835a a03c80;
T_BMP085=26.0C;
P_BMP085=99390Pa=745.4mmHg;
T_DS=26.0C.




