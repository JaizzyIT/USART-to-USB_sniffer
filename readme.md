# 3 channel USART to CDC USB sniffer #

USART sniffer based on Blue Pill dev board

## Description

Firmware based on FreeRTOS. 
All 3 UART channels are used (USART1, USART2, USART3) in Rx mode. All three channels receiving data simultaneously. 
Received data pass to onboard USB, working in CDC mode (as virtual COM port).  
Data shows in console in following format: `Un cccccccccccccccc XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX`  
Where:  
`n` - USART port number (1, 2 ,3),  
`c` - received byte as a char (16 in a row),  
`XX` - received byte as a hex (16 in a row).  

## Usage

To configure USART baud-rate - insert correct value to `USART1_BAUDRATE`, `USART2_BAUDRATE`, `USART3_BAUDRATE` macro definitions in *main.h* file.  
Connect to desired USART ports, connect Blue Pill via onboard USB to PC, open console for corresponding COM port.  
USART pinout for Blue Pill:  
   - USART1: Tx-PA9, Rx-PA10;  
   - USART2: Tx-PA2, Rx-PA3;  
   - USART1: Tx-PB10, Rx-PB11;  

## Notes

STM32F0, STM32F3, STM32F7, STM32H7, STM32L0, STM32L4 based devices support `ABR` - auto baud rate detection. Use following [application note](https://www.st.com/resource/en/application_note/an4908-stm32-usart-automatic-baud-rate-detection-stmicroelectronics.pdf) to implement it.

