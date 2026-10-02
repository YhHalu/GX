#include "uart/stm32f1xx_hal.h"
#include "motor_uart.h"
#include "motor_command.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

unsigned int test_uart_instance, test_gpio_instance;
uint32_t test_mask;
Motor_HandleTypeDef motorA, motorB;
static UART_HandleTypeDef *uart_handle;
static uint8_t *rx_buffer;
static uint8_t *pending_tx;
static char last_tx[MOTOR_COMMAND_REPLY_SIZE];
static unsigned int tx_aborts;
static HAL_StatusTypeDef transmit_status = HAL_OK;
static uint32_t tick;
static unsigned int button;
static unsigned int configured, abort_count;
static float first_target, second_target;
static HAL_StatusTypeDef update_status = HAL_BUSY;
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor)
{ if(motor==&motorA) first_target=0; else second_target=0; return HAL_OK; }

uint32_t HAL_GetTick(void) { return tick; }
unsigned int HAL_GPIO_ReadPin(void *port, uint16_t pin)
{ assert(port == GPIOB && pin == GPIO_PIN_2); return button; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOA);
    assert(gpio->Pin == GPIO_PIN_9 || gpio->Pin == GPIO_PIN_10);
}
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority)
{ assert(irq == USART1_IRQn && priority == 5 && subpriority == 0); }
void HAL_NVIC_EnableIRQ(uint32_t irq) { assert(irq == USART1_IRQn); }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart)
{
    assert(uart->Instance == USART1 && uart->Init.BaudRate == 115200);
    uart_handle = uart;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == uart_handle && size == 1);
    rx_buffer = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == uart_handle && size < sizeof(last_tx));
    if (transmit_status != HAL_OK) return transmit_status;
    assert(pending_tx == NULL);
    pending_tx = data;
    memcpy(last_tx, data, size);
    last_tx[size] = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ assert(uart == uart_handle); ++abort_count; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{ assert(uart == uart_handle); pending_tx = NULL; ++tx_aborts; return HAL_OK; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *uart) { assert(uart == uart_handle); }
HAL_StatusTypeDef Motor_PIDConfigureCounts(Motor_HandleTypeDef *motor, int8_t sign, const PID_Config *config)
{
    assert((motor == &motorA || motor == &motorB) && sign == 1);
    assert(config == &Motor_MeasuredCountsPID);
    ++configured;
    return HAL_OK;
}
HAL_StatusTypeDef Motor_Update(void) { return update_status; }
HAL_StatusTypeDef CSGO(float first, float second)
{ first_target = first; second_target = second; return HAL_OK; }

void Motor_GetFeedbackSnapshot(Motor_FeedbackSnapshot *s)
{ s->cps10[0]=s->cps10[1]=0; s->age_ms=0; s->valid_bits=3; }

static void receive(const char *text)
{
    while (*text)
    {
        *rx_buffer = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(uart_handle);
    }
}

static void drain_tx(void)
{
    unsigned int iteration;
    for (iteration = 0; iteration < 20; ++iteration)
    {
        if (pending_tx != NULL)
        {
            assert(strcmp((char *)pending_tx, last_tx) == 0);
            pending_tx = NULL;
            HAL_UART_TxCpltCallback(uart_handle);
        }
        Motor_UARTPoll();
    }
    assert(pending_tx == NULL);
}

static void request(const char *type, unsigned long seq, const char *args)
{
    char data[256];
    int n = snprintf(data, sizeof(data), "@2,%s,11111111111111111111111111111111,%lu%s%s", type, seq, args[0] ? "," : "", args);
    snprintf(data+n, sizeof(data)-(size_t)n, "*%04X\n", (unsigned int)Motor_CommandCRC((uint8_t*)data+1,(uint32_t)n-1U));
    receive(data);
}

static void press_start(void)
{
    button=0; ++tick; Motor_UARTPoll();
    tick+=30; Motor_UARTPoll();
    button=1; ++tick; Motor_UARTPoll();
    tick+=30; Motor_UARTPoll(); drain_tx();
}

int main(void)
{
    unsigned int count;
    assert(Motor_UARTInit() == HAL_ERROR);
    motorA.initialized = motorB.initialized = 1;
    assert(Motor_UARTInit() == HAL_OK && configured == 2);
    assert(Motor_UARTInit() == HAL_BUSY);
    Motor_UARTPoll(); assert(strstr(last_tx, ",READY,") != NULL); drain_tx();
    tick=10; update_status=HAL_OK;
    request("HELLO",0,""); request("STOP",1,""); Motor_UARTPoll(); drain_tx();
    assert(strstr(last_tx,"STOP,WAIT_START,0,0"));
    press_start(); ++tick; request("SET",2,"303,600"); Motor_UARTPoll();
    assert(fabsf(first_target-30.3f)<0.001f && strstr(last_tx,"SET,MOVING")); drain_tx();
    HAL_UART_ErrorCallback(uart_handle); Motor_UARTPoll();
    assert(first_target==0 && abort_count==1); drain_tx();
    ++tick; receive("\n"); request("STOP",3,""); Motor_UARTPoll(); drain_tx();
    press_start(); ++tick; request("SET",4,"1000,0"); Motor_UARTPoll();
    assert(first_target==100); drain_tx();
    for(count=0;count<300;count++) receive("9");
    Motor_UARTPoll(); assert(first_target==0 && abort_count==2); drain_tx();
    ++tick; receive("\n"); request("STOP",5,""); Motor_UARTPoll(); drain_tx();
    press_start(); ++tick; request("SET",6,"1000,0"); Motor_UARTPoll(); drain_tx();
    tick+=500; Motor_UARTPoll(); assert(first_target==0); drain_tx(); assert(strstr(last_tx,"LINK_TIMEOUT"));
    ++tick; request("STOP",7,""); Motor_UARTPoll(); drain_tx();
    press_start(); ++tick; request("SET",8,"1000,0"); Motor_UARTPoll(); drain_tx();
    update_status=HAL_TIMEOUT; Motor_UARTPoll(); assert(first_target==0); drain_tx();
    update_status=HAL_OK;
    ++tick; request("STOP",9,""); Motor_UARTPoll(); drain_tx();
    press_start(); ++tick; request("SET",10,"1000,0");
    tick+=100; Motor_UARTPoll(); assert(first_target==0); drain_tx();
    ++tick; receive("\n"); request("STOP",11,""); Motor_UARTPoll(); drain_tx(); press_start();
    for(count=0;count<10;count++) { request("SET",12+count,"1000,0"); Motor_UARTPoll(); }
    assert(first_target==0); drain_tx(); assert(strstr(last_tx,"TX_BACKPRESSURE"));
    ++tick; request("STOP",30,""); Motor_UARTPoll(); drain_tx(); press_start();
    request("SET",31,"1000,0"); Motor_UARTPoll(); assert(pending_tx != NULL && first_target==100);
    tick+=100; Motor_UARTPoll(); assert(tx_aborts==1 && first_target==0); drain_tx();
    transmit_status=HAL_BUSY;
    ++tick; request("STOP",32,""); Motor_UARTPoll();
    tick+=100; Motor_UARTPoll(); assert(tx_aborts>=2 && first_target==0);
    transmit_status=HAL_OK; drain_tx();
    puts("PASS: UART init, v2 RX, stable async buffers, RX overflow/error/staleness, motor/lease stop, critical backpressure, TX stalls");
    return 0;
}
