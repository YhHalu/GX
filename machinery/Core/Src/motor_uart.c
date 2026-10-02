#include "motor_uart.h"
#include "main.h"
#include "motor_command.h"
#include "motor.h"
#include <string.h>

#define RX_CAPACITY 256U
#define TX_CAPACITY 8U

typedef struct {
    uint32_t tick;
    uint8_t byte;
} ReceivedByte;

static UART_HandleTypeDef command_uart;
static uint8_t received_byte;
static volatile ReceivedByte rx_queue[RX_CAPACITY];
static volatile uint32_t rx_head, rx_tail;
static volatile uint8_t rx_failed;
static char tx_queue[TX_CAPACITY][MOTOR_COMMAND_REPLY_SIZE];
static uint32_t tx_head, tx_tail;
static uint8_t tx_active;
static uint8_t tx_overflow;
static volatile uint8_t tx_done;
static uint8_t uart_ready;
static char status_pending[MOTOR_COMMAND_REPLY_SIZE];
static char status_active[MOTOR_COMMAND_REPLY_SIZE];
static uint8_t status_waiting, motor_healthy = 1U;
static uint32_t tx_start_tick, busy_tick;
static uint8_t busy_waiting;

static int queue_status(const char *message)
{
    size_t size = strlen(message);
    if (size >= MOTOR_COMMAND_REPLY_SIZE) return 0;
    memcpy(status_pending, message, size + 1U);
    status_waiting = 1U;
    return 1;
}

static int queue_reply(const char *message)
{
    size_t size = strlen(message);
    uint32_t next = (tx_head + 1U) % TX_CAPACITY;
    if (next == tx_tail || size >= MOTOR_COMMAND_REPLY_SIZE)
    {
        tx_overflow = 1U;
        return 0;
    }
    memcpy(tx_queue[tx_head], message, size + 1U);
    tx_head = next;
    return 1;
}

HAL_StatusTypeDef Motor_UARTInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    unsigned int index;
    Motor_HandleTypeDef *motors[2] = {&motorA, &motorB};
    if (uart_ready) return HAL_BUSY;
    if (!motorA.initialized || !motorB.initialized) return HAL_ERROR;
    for (index = 0U; index < 2U; ++index)
    {
        if (Motor_PIDConfigureCounts(motors[index], 1, &Motor_MeasuredCountsPID) != HAL_OK)
            return HAL_ERROR;
    }
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_DISABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    command_uart.Instance = USART1;
    command_uart.Init.BaudRate = 115200U;
    command_uart.Init.WordLength = UART_WORDLENGTH_8B;
    command_uart.Init.StopBits = UART_STOPBITS_1;
    command_uart.Init.Parity = UART_PARITY_NONE;
    command_uart.Init.Mode = UART_MODE_TX_RX;
    command_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    command_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&command_uart) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(&command_uart, &received_byte, 1U) != HAL_OK)
        return HAL_ERROR;
    uart_ready = 1U;
    Motor_CommandInit(queue_reply, queue_status);
    return HAL_OK;
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&command_uart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t next;
    if (uart != &command_uart) return;
    next = (rx_head + 1U) % RX_CAPACITY;
    if (next == rx_tail) rx_failed = 1U;
    else
    {
        rx_queue[rx_head].tick = HAL_GetTick();
        rx_queue[rx_head].byte = received_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(uart, &received_byte, 1U) != HAL_OK) rx_failed = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &command_uart) rx_failed = 1U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &command_uart) tx_done = 1U;
}

void Motor_UARTPoll(void)
{
    uint32_t count;
    uint32_t mask;
    HAL_StatusTypeDef status;
    if (!uart_ready) return;
    status = Motor_Update();
    if (status != HAL_BUSY) motor_healthy = (uint8_t)(status == HAL_OK);
    if (status == HAL_ERROR || status == HAL_TIMEOUT)
        Motor_CommandFault("MOTOR_SAMPLE_OR_FEEDBACK");
    /* Enforce the current lease before draining bytes with older timestamps. */
    Motor_CommandPoll(HAL_GetTick());
    if (tx_active && tx_done)
    {
        tx_done = 0U;
        if (tx_active == 1U) tx_tail = (tx_tail + 1U) % TX_CAPACITY;
        tx_active = 0U;
    }
    if ((tx_active && (uint32_t)(HAL_GetTick() - tx_start_tick) >= 100U) ||
        (busy_waiting && (uint32_t)(HAL_GetTick() - busy_tick) >= 100U))
    {
        (void)HAL_UART_AbortTransmit(&command_uart);
        tx_active = tx_done = busy_waiting = 0U;
        tx_tail = tx_head;
        Motor_CommandFault("UART_TX_TIMEOUT");
    }
    if (rx_failed)
    {
        Motor_CommandFault("UART_RX");
        mask = __get_PRIMASK();
        __disable_irq();
        (void)HAL_UART_AbortReceive(&command_uart);
        __HAL_UART_CLEAR_OREFLAG(&command_uart);
        rx_tail = rx_head;
        rx_failed = 0U;
        if (HAL_UART_Receive_IT(&command_uart, &received_byte, 1U) != HAL_OK)
            rx_failed = 1U;
        __set_PRIMASK(mask);
    }
    Motor_CommandSetHealthy((uint8_t)(motor_healthy && !rx_failed && !tx_overflow && !busy_waiting));
    for (count = 0U; count < RX_CAPACITY && rx_tail != rx_head && !rx_failed; ++count)
    {
        uint32_t received_tick = rx_queue[rx_tail].tick;
        uint8_t byte = rx_queue[rx_tail].byte;
        rx_tail = (rx_tail + 1U) % RX_CAPACITY;
        if ((uint32_t)(HAL_GetTick() - received_tick) >= MOTOR_COMMAND_PARTIAL_MS)
        {
            Motor_CommandFault("STALE_RX");
            rx_failed = 1U;
            break;
        }
        Motor_CommandReceive(byte, received_tick);
    }
    /* Drain existing requests before granting permission to future frames. */
    Motor_CommandButtonPoll((uint8_t)(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,
        START_BUTTON_Pin) == GPIO_PIN_SET), HAL_GetTick());
    Motor_CommandPoll(HAL_GetTick());
    if (tx_overflow && (tx_head + 1U) % TX_CAPACITY != tx_tail)
    {
        tx_overflow = 0U;
        Motor_CommandFault("TX_BACKPRESSURE");
    }
    if (!tx_active && (tx_tail != tx_head || status_waiting))
    {
        uint8_t kind;
        char *message;
        tx_done = 0U;
        kind = tx_tail != tx_head ? 1U : 2U;
        if (kind == 2U) memcpy(status_active, status_pending, sizeof(status_active));
        message = kind == 1U ? tx_queue[tx_tail] : status_active;
        status = HAL_UART_Transmit_IT(&command_uart, (uint8_t *)message, (uint16_t)strlen(message));
        if (status == HAL_OK)
        {
            tx_active = kind;
            tx_start_tick = HAL_GetTick();
            busy_waiting = 0U;
            if (kind == 2U) status_waiting = 0U;
        }
        else if (status == HAL_BUSY)
        {
            if (!busy_waiting) { busy_tick = HAL_GetTick(); busy_waiting = 1U; }
        }
        else
        {
            tx_tail = tx_head;
            busy_waiting = 0U;
            Motor_CommandSetHealthy(0U);
            Motor_CommandFault("UART_TX");
        }
    }
}
