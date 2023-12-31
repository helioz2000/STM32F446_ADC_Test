#ADC test code
to develop ADC functionality for power meter application

#Hardware
This project is designed to run on a Nucleo 64 board. The board is marked **NUCLEO-F446RE** and is based on the STM32F446 MCU.

A custom PCB will be connected to the Nucleo development board. The PCB will house an interface to 3 Current Transformers and 1 Voltage transformer for the purpose bringing AC Voltage and up to 3 current signals to the Analog inputs of the MCU via OpAmp.

#Analog Inputs
- **IN0** - ADC1 - AC Voltage
- **IN1** - ADC2 - AC Current 1
- **IN10** - ADC1 - AC Current 2
- **IN11** - ADC2 - AC Current 3

#Pinout
- IN0 : PA0 : CN8-1 : CN7-28 : Arduino A0 V1
- IN1 : PA1 : CN8-2 : CN7-30 : Arduino A1 I2
- IN10 : PC0 : CN8-6 : CN7-38 : Arduino A5 I1
- IN11 : PC1 : CN8-5 : CN7-36 : Arduino A4 I3

#Timer 2 usage
This timer triggers the ADC measurements. 

- TIM2 - APB1 - 90MHz CLK = 11.11111ns / pulse
- PSC 0 (No prescale)
- ARR 2250 = 25us (theoretically)
- The first test board requires ARR 2286  (91.45MHz), the second board ARR is 2275 to achieve 25us.

ADC measurement every 25us allows for 800 measurements per 20ms (one 50Hz cycle) providing a resolution better than 0.5 degrees of phase.

To enable accurate analysis of each data set we look at **840** data samples covering **21ms**. Each data set will capture a full cycle at a mains frequency as low as  **47.62 Hz**. 

#Timer 3 usage
This timer is used to trigger energy (VAh & Wh) measurements by itegrating the VA and W readings over time and storing them in VAh and Wh variables.

- TIM3 - APB1 - 90MHz CLK = 11.11111ns / pulse
- PSC 9000 (100us)
- ARR 1000 = (100ms)


To produce a timer call once a second use the following settings:
- PSC 9000 (100us)
- ARR 10000 (1s)

#ADC usage
PCLK2 (90MHz) / 8 = 11.25 Mhz ADC clock = 88.88ns

| time | purpose |
| --- | --- |
| 56 | CH0 Sample |
| 15 | CH0 Conversion |
| 56 | CH1 Sample |
| 15 | CH1 Conversion |
| **60** | **Total time** |
142 cycles = 12.6us = time to sample 2 channels on a single ADC.

In this example ADC1 and ADC2 are used to sample simultaneously resulting in a total of **4 channels** sampled every **12.6us**

**Note:** the ADC is capable of much faster sampling but to transfer the data to memory takes time and can result in an overrun error if the next conversion is started before the data is read. 


#WiFi Interface
ESP-01 connected to USART3

| Pin | Name | ESP |
| --- | --- | --- |
| PC10 | USART3_TX | ESP01 RX |
| PC5 | USART3_RX | ESP01 TX |
| PA15 | ESP01_EN | Enable |
| PB7 | ESP01_RST | Reset |

Baudrate: on V1.0 of the ESP firmare the baudrate after reset is 76800 until startup has finished and it changes to 115200.