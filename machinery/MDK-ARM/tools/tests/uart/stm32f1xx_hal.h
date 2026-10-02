#ifndef TEST_UART_HAL_H
#define TEST_UART_HAL_H

#include "../stm32f1xx_hal.h"

typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
} UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; } UART_HandleTypeDef;

extern unsigned int test_uart_instance, test_gpio_instance;
#define USART1 (&test_uart_instance)
#define GPIOA (&test_gpio_instance)
#define GPIOB ((void *)((char *)&test_gpio_instance + 1))
#define GPIO_PIN_2 4U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_MODE_AF_PP 2U
#define GPIO_MODE_INPUT 0U
#define GPIO_SPEED_FREQ_HIGH 3U
#define GPIO_PULLUP 1U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 12U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define USART1_IRQn 37U
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_AFIO_REMAP_USART1_DISABLE() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))

void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
unsigned int HAL_GPIO_ReadPin(void *, uint16_t);
void HAL_NVIC_SetPriority(uint32_t, uint32_t, uint32_t);
void HAL_NVIC_EnableIRQ(uint32_t);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
void HAL_UART_IRQHandler(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

#endif
