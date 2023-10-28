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
- IN0 : PA0 : CN8-1 : CN7-28 : Arduino A0 
- IN1 : PA1 : CN8-2 : CN7-30 : Arduino A1 
- IN10 : PC0 : CN8-6 : CN7-38 : Arduino A5
- IN11 : PC1 : CN8-5 : CN7-36 : Arduino A4

#Timer usage
TIM2 - APB1 - 90MHz CLK = 11.11111ns / pulse
PSC 0 (No prescale)
ARR 2250 = 25us (theoretically)
The test board needed ARR 2286 to achieve 25us (91.45MHz)

ADC measurement every 25us allows for 800 measurements per 20ms (one 50Hz cycle) providing a reading better than 0.5 degrees of phase. 

#ADC usage
PCLK2 (90MHz) / 8 = 11.25 Mhz ADC clock = 88.88ns

| time | purpose |
| --- | --- |
| 15 | CH0 Sample |
| 15 | CH0 Conversion |
| 15 | CH1 Sample |
| 15 | CH1 Conversion |
| **60** | **Total time** |
60 cycles = 5.3us = time to sample 2 channels on a single ADC.

In this example ADC1 and ADC2 are used to sample simultaneously resulting in a total of **4 channels** sampled every **5.3us**

**Note:** the ADC is capable of much faster sampling but to transfer the data to memory takes time and can result in an overrun error it the next conversion is started before the data is read. 


