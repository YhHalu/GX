#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdint.h>

#define MOTOR_COMMAND_PARTIAL_MS 100U
#define MOTOR_COMMAND_LEASE_MS 500U
#define MOTOR_COMMAND_STATUS_MS 100U
#define MOTOR_COMMAND_FRAME_MAX 192U
#define MOTOR_COMMAND_REPLY_SIZE (MOTOR_COMMAND_FRAME_MAX + 1U)
#define MOTOR_COMMAND_FIRMWARE "GXUART2_BTN"
#define MOTOR_COMMAND_BUTTON_MS 30U

typedef int (*Motor_CommandSend)(const char *message);

void Motor_CommandInit(Motor_CommandSend send, Motor_CommandSend status);
void Motor_CommandSetHealthy(uint8_t healthy);
/* SW2/PB2 is active high. Poll from the same foreground owner as UART. */
void Motor_CommandButtonPoll(uint8_t pressed, uint32_t tick);
uint16_t Motor_CommandCRC(const uint8_t *bytes, uint32_t size);
void Motor_CommandReceive(uint8_t byte, uint32_t tick);
void Motor_CommandPoll(uint32_t tick);
void Motor_CommandFault(const char *reason);

#endif
