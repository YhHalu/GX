#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Two-input H-bridge: IN1 PWM / IN2 low = positive direction.
 * IN1 low / IN2 PWM = negative direction; both low = coast.
 * Both high = brake (AT8236 truth table).
 * Zero-initialize each handle. Call from one foreground context only.
 */
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t in1_channel;
    uint32_t in2_channel;
    uint8_t initialized;
} Motor_HandleTypeDef;

/* Board motors: motorA = CN1, motorB = CN2. */
extern Motor_HandleTypeDef motorA;
extern Motor_HandleTypeDef motorB;

/* Call once AFTER CubeMX MX_TIM2/3/4_Init. Starts at zero duty.
 * Uses CubeMX handles; does not configure clocks, pins or timer modes. */
HAL_StatusTypeDef Motor_SystemInit(void);
/* Call repeatedly in the main loop; samples encoders every >=10ms.
 * HAL_BUSY: not due; HAL_TIMEOUT: invalid sample; HAL_OK: new sample. */
HAL_StatusTypeDef Motor_Update(void);

/* RPM closed loop for motorA/motorB only. Configure after SystemInit.
 * CPR is output-shaft counts/rev AFTER quadrature x4; sign is +1/-1.
 * config output_min must be 0; output_max in (0,1000] permille.
 * Configure stops the selected motor; valid CPR and actual tuned gains
 * are required. No automatic enable or assumed motor specifications.
 */
HAL_StatusTypeDef Motor_PIDConfigure(Motor_HandleTypeDef *motor, uint32_t cpr,
                                     int8_t sign, const PID_Config *config);
/* Signed target RPM. 0 disables PID and coasts. A running target cannot
 * change sign: command 0, wait for the load to slow, then reverse.
 * Overspeed reduces PWM to zero; PID does not reverse torque to brake.
 */
HAL_StatusTypeDef Motor_SetTargetRPM(Motor_HandleTypeDef *motor, float rpm);
HAL_StatusTypeDef Motor_PIDConfigureCounts(Motor_HandleTypeDef *motor,
                                           int8_t sign, const PID_Config *config);
HAL_StatusTypeDef Motor_SetTargetCountsPerSecond(Motor_HandleTypeDef *motor,
                                                float counts_per_second);
HAL_StatusTypeDef Motor_SetTargetsCountsPerSecond(float cn1, float cn2);
HAL_StatusTypeDef Motor_SetFeedbackTimeout(Motor_HandleTypeDef *motor,
                                          uint32_t timeout_ms);
HAL_StatusTypeDef Motor_PIDDisable(Motor_HandleTypeDef *motor);
uint8_t Motor_PIDIsEnabled(const Motor_HandleTypeDef *motor);

HAL_StatusTypeDef Motor_SetTargetsRPM(float cn1_rpm, float cn2_rpm);
HAL_StatusTypeDef CSGO(float cn1_speed_percent, float cn2_speed_percent);

/* Signed integer PWM percentages, clamped to [-100, 100].
 * Positive/negative select direction; zero coasts. Not RPM or PID.
 * Call after both Motor_Init calls succeed. */
HAL_StatusTypeDef Motor_SetDutyPercent(int32_t cn1_percent, int32_t cn2_percent);

/* Call AFTER MX_TIMx_Init(). Timer must be TIM2/3/4, up-counting,
 * internally clocked, with ARR in [1, 65534]. CubeMX must configure both
 * channels as PWM mode 1, active high. Channels must be unused.
 * Starts both channels at zero duty; does not change PSC/ARR or GPIOs.
 */
HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *motor,
                            TIM_HandleTypeDef *timer,
                            uint32_t in1_channel, uint32_t in2_channel);

/* Signed duty in permille, clamped to [-1000, 1000]. This is open-loop
 * duty control, not measured RPM. 500 = 50%, -500 = reverse 50%.
 * Updates take effect at the next timer update event (within one period).
 * Before reversing a moving load, coast until it has slowed down.
 */
HAL_StatusTypeDef Motor_SetSpeed(Motor_HandleTypeDef *motor, int32_t duty);
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor);
HAL_StatusTypeDef Motor_Brake(Motor_HandleTypeDef *motor);
HAL_StatusTypeDef Motor_DeInit(Motor_HandleTypeDef *motor);

/* Four-edge counts, not single-channel pulses. Zero CPR means uncalibrated. */
typedef struct {
    uint16_t previous;
    int8_t sign;
    uint32_t counts_per_turn;
    int32_t delta;
    int64_t position;
    float counts_per_second;
    float rpm;
    uint8_t valid;
} Encoder_State;

extern Encoder_State encoderA;
extern Encoder_State encoderB;

/* Foreground-only snapshot. CPS is signed quadrature counts/s times ten. */
typedef struct {
    int32_t cps10[2];
    uint32_t age_ms;
    uint8_t valid_bits;
} Motor_FeedbackSnapshot;
void Motor_GetFeedbackSnapshot(Motor_FeedbackSnapshot *snapshot);

void Encoder_Reset(Encoder_State *s, uint16_t counter);
/* Call frequently enough that actual movement is <32768 counts per sample.
 * Gap >100 ms or exactly half a counter range invalidates this sample.
 */
int Encoder_Sample(Encoder_State *s, uint16_t counter, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif
#endif
