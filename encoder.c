#include "encoder.h"
// Private variables
static UART_HandleTypeDef *encoderUART = NULL;
static volatile uint8_t rxComplete = 0;
static volatile uint8_t rxError = 0;
static uint8_t rxData[2];
static uint16_t lastGoodPosition = 0;


extern TIM_HandleTypeDef htim2;

static inline uint32_t getTime_us(void) {
   return __HAL_TIM_GET_COUNTER(&htim2);
}


// Checksum verification
static uint8_t AMT21_VerifyChecksum(uint8_t high, uint8_t low) {
   uint8_t k1_received = (high >> 7) & 1;
   uint8_t k0_received = (high >> 6) & 1;
   uint16_t data = ((high & 0x3F) << 8) | low;
   uint8_t k1_calc = 0;
   k1_calc ^= (data >> 13) & 1;
   k1_calc ^= (data >> 11) & 1;
   k1_calc ^= (data >> 9) & 1;
   k1_calc ^= (data >> 7) & 1;
   k1_calc ^= (data >> 5) & 1;
   k1_calc ^= (data >> 3) & 1;
   k1_calc ^= (data >> 1) & 1;
   k1_calc = !k1_calc;
   uint8_t k0_calc = 0;
   k0_calc ^= (data >> 12) & 1;
   k0_calc ^= (data >> 10) & 1;
   k0_calc ^= (data >> 8) & 1;
   k0_calc ^= (data >> 6) & 1;
   k0_calc ^= (data >> 4) & 1;
   k0_calc ^= (data >> 2) & 1;
   k0_calc ^= (data >> 0) & 1;
   k0_calc = !k0_calc;
   return (k1_received == k1_calc) && (k0_received == k0_calc);
}


// RS485 direction control
static void RS485_TransmitMode(void) {
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
   for(volatile int i = 0; i < 10; i++) __NOP();
}
static void RS485_ReceiveMode(void) {
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
   for(volatile int i = 0; i < 10; i++) __NOP();
}
// DMA callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART1) {
       rxComplete = 1;
   }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART1) {
       rxError = 1;
   }
}


// Main read function (private)
static HAL_StatusTypeDef AMT212B_ReadPosition(UART_HandleTypeDef *huart, uint16_t *position) {
   uint8_t cmd = 0x54;
   uint32_t tickstart;
   HAL_StatusTypeDef status;
   rxComplete = 0;
   rxError = 0;
   rxData[0] = rxData[1] = 0x00;
   RS485_ReceiveMode();
   for (volatile int i = 0; i < 50; ++i) __NOP();
   __HAL_UART_FLUSH_DRREGISTER(huart);
   RS485_TransmitMode();
   for (volatile int i = 0; i < 50; ++i) __NOP();
   status = HAL_UART_Transmit(huart, &cmd, 1, 100);
   if (status != HAL_OK) {
       RS485_ReceiveMode();
       return status;
   }
   tickstart = HAL_GetTick();
   while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
       if ((HAL_GetTick() - tickstart) > 100) {
           RS485_ReceiveMode();
           return HAL_TIMEOUT;
       }
   }
   RS485_ReceiveMode();
   for (volatile int i = 0; i < 30; ++i) __NOP();
   status = HAL_UART_Receive_DMA(huart, rxData, 2);
   if (status != HAL_OK) {
       return status;
   }
   tickstart = HAL_GetTick();
   while (!rxComplete && !rxError) {
       if ((HAL_GetTick() - tickstart) > 50) {
           HAL_UART_DMAStop(huart);
           break;
       }
   }
   uint16_t transferred = 2;
   if (!rxComplete) {
       transferred = 2 - huart->hdmarx->Instance->CNDTR;
   }
   if (transferred >= 2) {
       if (!AMT21_VerifyChecksum(rxData[0], rxData[1])) {
           return HAL_ERROR;
       }
       uint16_t data = ((rxData[0] & 0x3F) << 8) | rxData[1];
       *position = data;
       return HAL_OK;
   }
   return HAL_TIMEOUT;
}


// PUBLIC FUNCTIONS
void Encoder_Init(UART_HandleTypeDef *huart) {
   encoderUART = huart;
   lastGoodPosition = 0;
   RS485_ReceiveMode();
   HAL_Delay(10);
}


uint16_t getEncoderPosition(void) {
   uint16_t position = 0;
   HAL_StatusTypeDef status;
   if (encoderUART == NULL) {
       return 0;
   }
   status = AMT212B_ReadPosition(encoderUART, &position);
   if (status == HAL_OK) {
       lastGoodPosition = position;
       return position;
   } else {
       return lastGoodPosition;
   }
}


HAL_StatusTypeDef getEncoderPosition_WithStatus(uint16_t *position) {
   if (encoderUART == NULL) {
       return HAL_ERROR;
   }
   HAL_StatusTypeDef status = AMT212B_ReadPosition(encoderUART, position);
   if (status == HAL_OK) {
       lastGoodPosition = *position;
   }
   return status;
}


EncoderData_t getEncoderPosition_WithTiming(void) {
   EncoderData_t data;
   data.position = 0;
   data.readTime_us = 0;
   if (encoderUART == NULL) {
       data.status = HAL_ERROR;
       return data;
   }
   uint32_t startTime = getTime_us();
   data.position = getEncoderPosition();  // always reliable
   uint32_t endTime = getTime_us();
   data.readTime_us = endTime - startTime;
   if (data.status == HAL_OK) {
       lastGoodPosition = data.position;
   }
   return data;
}

