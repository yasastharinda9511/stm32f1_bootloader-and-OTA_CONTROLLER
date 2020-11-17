# stm32f1_bootloader and OTA CONTROLLER

# Introduction

When we are designing the new obd board we had a problem. The problem is, suppose if we want to update our device firmware with
more complicated or more efficient firmware than current version then how we are going to inform the client. If client get the information
how he can engage with firmware update. I think it somewhat complicated if we inform the client, you have a new firmware update and come to
our company asap to update your device. Actually, which is not the exact way to address the problem. Then we research about the how modern
embedded systems get their updates automatically. We found a solution but these solutions are more advance and secure. Therefore, we got the
overall idea about what OTA programming is. Then we came up with our solution using our limited resources. But main challenge was, we had to
add another microcontroller and memory element to store the updated firmware and the backup the currently running program.

## BootLoader

Microcontrollers provide their own datasheet about how to access the flash memory of the microcontrollers and what are the specific steps to be followed. AN2606 application
note datasheet published by Stmicrocontrollers provide all the data. For stm32f103 series UART flash programming is allowed by the manufacturer. Therefore, we have to program
the flash memory of the microcontroller through the UART1. When accessing the flash memory of the microcontroller, BOOT0 and BOOT1 pin should be 1 and 0 respectively.

floaw chart of the flash memeory access in STM32F1

![Screenshot (523)](https://user-images.githubusercontent.com/37435024/99406317-c586df00-2913-11eb-90ef-cd961db1406c.png)

our booloader code is running inside a stm32f030f4. The pin diagram of the microcontroller is like below



## OTA CONTROLLER ARCHITECTURE

![Screenshot (519)](https://user-images.githubusercontent.com/37435024/99390162-2c010280-28fe-11eb-8bef-72d04ce9ab02.png)

## How OTA works

  ![Screenshot (521)](https://user-images.githubusercontent.com/37435024/99393761-cfa0e180-2903-11eb-8009-724a9dc345b3.png)

## Our testbed


