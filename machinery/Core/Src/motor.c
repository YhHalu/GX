#include "motor.h"
#include "tim.h"
#include <float.h>

Motor_HandleTypeDef motorA = {0};
Motor_HandleTypeDef motorB = {0};
Encoder_State encoderA;
Encoder_State encoderB;
static uint32_t encoderTick;
static uint8_t systemReady;

typedef enum {
    MOTOR_SPEED_RPM,
    MOTOR_SPEED_COUNTS
} Motor_SpeedUnit;

typedef struct {
    PID_Controller pid;
    float target;
    Motor_SpeedUnit unit;
    uint32_t feedback_timeout_ms;
    uint32_t no_feedback_ms;
    uint32_t cpr;
    int8_t sign;
    uint8_t configured;
    uint8_t enabled;
} Motor_SpeedControl;

static Motor_SpeedControl speedControl[2];
static HAL_StatusTypeDef apply_duty(Motor_HandleTypeDef *motor, int32_t duty);
static HAL_StatusTypeDef speed_update(uint32_t elapsed_ms);

static Motor_SpeedControl *speed_control(const Motor_HandleTypeDef *motor)
{
    if (motor == &motorA) return &speedControl[0];
    if (motor == &motorB) return &speedControl[1];
    return NULL;
}

static void disable_speed_control(Motor_HandleTypeDef *motor)
{
    Motor_SpeedControl *control = speed_control(motor);
    if (control != NULL)
    {
        control->enabled = 0U;
        control->target = 0.0f;
        control->no_feedback_ms = 0U;
        PID_Reset(&control->pid);
    }
}

HAL_StatusTypeDef Motor_SystemInit(void)
{
    HAL_StatusTypeDef status;
    if (motorA.initialized || motorB.initialized) return HAL_BUSY;
    disable_speed_control(&motorA);
    disable_speed_control(&motorB);
    speedControl[0].configured = 0U;
    speedControl[1].configured = 0U;
    systemReady = 0U;
    status = Motor_Init(&motorA, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
    if (status != HAL_OK) return status;
    status = Motor_Init(&motorB, &htim2, TIM_CHANNEL_4, TIM_CHANNEL_3);
    if (status != HAL_OK) goto fail_pwm;
    status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (status != HAL_OK) goto fail_pwm;
    status = HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        goto fail_pwm;
    }
    Encoder_Reset(&encoderA, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3));
    Encoder_Reset(&encoderB, (uint16_t)__HAL_TIM_GET_COUNTER(&htim4));
    encoderTick = HAL_GetTick();
    systemReady = 1U;
    return HAL_OK;
fail_pwm:
    if (motorB.initialized) (void)Motor_DeInit(&motorB);
    (void)Motor_DeInit(&motorA);
    return status;
}

HAL_StatusTypeDef Motor_Update(void)
{
    uint32_t now, elapsed;
    int a, b;
    if (!systemReady || !motorA.initialized || !motorB.initialized)
    {
        if (speedControl[0].enabled) (void)Motor_Stop(&motorA);
        if (speedControl[1].enabled) (void)Motor_Stop(&motorB);
        return HAL_ERROR;
    }
    now = HAL_GetTick();
    elapsed = now - encoderTick;
    if (elapsed < 10U) return HAL_BUSY;
    encoderTick = now;
    a = Encoder_Sample(&encoderA, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3), elapsed);
    b = Encoder_Sample(&encoderB, (uint16_t)__HAL_TIM_GET_COUNTER(&htim4), elapsed);
    if (!a || !b)
    {
        if (speedControl[0].enabled) (void)Motor_Stop(&motorA);
        if (speedControl[1].enabled) (void)Motor_Stop(&motorB);
        return HAL_TIMEOUT;
    }
    return speed_update(elapsed);
}

void Motor_GetFeedbackSnapshot(Motor_FeedbackSnapshot *snapshot)
{
    Encoder_State *encoders[2] = {&encoderA, &encoderB};
    unsigned int i;
    if (snapshot == NULL) return;
    snapshot->age_ms = HAL_GetTick() - encoderTick;
    snapshot->valid_bits = 0U;
    for (i = 0U; i < 2U; ++i)
    {
        float value = encoders[i]->counts_per_second;
        snapshot->cps10[i] = 0;
        if (systemReady && snapshot->age_ms <= 100U && encoders[i]->valid &&
            value == value && value > -214748300.0f && value < 214748300.0f)
        {
            float scaled = value * 10.0f;
            snapshot->cps10[i] = (int32_t)(scaled + (scaled < 0.0f ? -0.5f : 0.5f));
            snapshot->valid_bits |= (uint8_t)(1U << i);
        }
    }
}

static int channel_valid(uint32_t channel)
{
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
           channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4;
}

static int motor_valid(const Motor_HandleTypeDef *motor)
{
    return motor != NULL && motor->initialized && motor->timer != NULL;
}

/* Prevent an overflow from latching only one of the two new CCR values.
 * UDIS does not stop the counter. Preserve the caller's interrupt mask.
 */
static void write_pair(Motor_HandleTypeDef *motor, uint32_t in1, uint32_t in2)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t saved_udis;
    __disable_irq();
    saved_udis = motor->timer->Instance->CR1 & TIM_CR1_UDIS;
    SET_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in1_channel, in1);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in2_channel, in2);
    if (saved_udis == 0U)
    {
        CLEAR_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    }
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *motor,
                            TIM_HandleTypeDef *timer,
                            uint32_t in1_channel, uint32_t in2_channel)
{
    HAL_StatusTypeDef status;
    uint32_t arr;

    if (motor == NULL || timer == NULL || motor->initialized ||
        !channel_valid(in1_channel) || !channel_valid(in2_channel) ||
        in1_channel == in2_channel)
    {
        return HAL_ERROR;
    }
    if (timer->Instance != TIM2 && timer->Instance != TIM3 &&
        timer->Instance != TIM4)
    {
        return HAL_ERROR;
    }
    arr = __HAL_TIM_GET_AUTORELOAD(timer);
    if (arr < 1U || arr > 65534U ||
        (timer->Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS)) != 0U ||
        (timer->Instance->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U ||
        HAL_TIM_GetChannelState(timer, in1_channel) != HAL_TIM_CHANNEL_STATE_READY ||
        HAL_TIM_GetChannelState(timer, in2_channel) != HAL_TIM_CHANNEL_STATE_READY)
    {
        return HAL_ERROR;
    }

    /* Load zero immediately, even if other channels already run on TIM2.
     * Do not issue UG: that would disturb the other motor's PWM period.
     */
    __HAL_TIM_DISABLE_OCxPRELOAD(timer, in1_channel);
    __HAL_TIM_DISABLE_OCxPRELOAD(timer, in2_channel);
    __HAL_TIM_SET_COMPARE(timer, in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(timer, in2_channel, 0U);
    __HAL_TIM_ENABLE_OCxPRELOAD(timer, in1_channel);
    __HAL_TIM_ENABLE_OCxPRELOAD(timer, in2_channel);

    status = HAL_TIM_PWM_Start(timer, in1_channel);
    if (status != HAL_OK) return status;
    status = HAL_TIM_PWM_Start(timer, in2_channel);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(timer, in1_channel);
        return status;
    }

    motor->timer = timer;
    motor->in1_channel = in1_channel;
    motor->in2_channel = in2_channel;
    motor->initialized = 1U;
    return HAL_OK;
}

static HAL_StatusTypeDef apply_duty(Motor_HandleTypeDef *motor, int32_t duty)
{
    uint32_t ticks;
    uint32_t magnitude;
    uint32_t arr;
    if (!motor_valid(motor)) return HAL_ERROR;
    arr = __HAL_TIM_GET_AUTORELOAD(motor->timer);
    if (arr < 1U || arr > 65534U) return HAL_ERROR;
    if (duty > 1000) duty = 1000;
    if (duty < -1000) duty = -1000;
    magnitude = (uint32_t)(duty < 0 ? -duty : duty);
    ticks = ((arr + 1U) * magnitude + 500U) / 1000U;
    write_pair(motor, duty > 0 ? ticks : 0U, duty < 0 ? ticks : 0U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetSpeed(Motor_HandleTypeDef *motor, int32_t duty)
{
    disable_speed_control(motor);
    return apply_duty(motor, duty);
}

HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor)
{
    disable_speed_control(motor);
    if (!motor_valid(motor)) return HAL_ERROR;
    write_pair(motor, 0U, 0U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetDutyPercent(int32_t cn1_percent, int32_t cn2_percent)
{
    HAL_StatusTypeDef status;
    if (!motor_valid(&motorA) || !motor_valid(&motorB)) return HAL_ERROR;
    /* Clamp before multiplying, including for INT32_MIN/INT32_MAX. */
    if (cn1_percent > 100) cn1_percent = 100;
    if (cn1_percent < -100) cn1_percent = -100;
    if (cn2_percent > 100) cn2_percent = 100;
    if (cn2_percent < -100) cn2_percent = -100;
    status = Motor_SetSpeed(&motorA, cn1_percent * 10);
    if (status != HAL_OK) return status;
    return Motor_SetSpeed(&motorB, cn2_percent * 10);
}

HAL_StatusTypeDef Motor_Brake(Motor_HandleTypeDef *motor)
{
    uint32_t arr;
    disable_speed_control(motor);
    if (!motor_valid(motor)) return HAL_ERROR;
    arr = __HAL_TIM_GET_AUTORELOAD(motor->timer);
    if (arr < 1U || arr > 65534U) return HAL_ERROR;
    write_pair(motor, arr + 1U, arr + 1U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_DeInit(Motor_HandleTypeDef *motor)
{
    HAL_StatusTypeDef first;
    HAL_StatusTypeDef second;
    disable_speed_control(motor);
    if (!motor_valid(motor)) return HAL_ERROR;
    __HAL_TIM_DISABLE_OCxPRELOAD(motor->timer, motor->in1_channel);
    __HAL_TIM_DISABLE_OCxPRELOAD(motor->timer, motor->in2_channel);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in2_channel, 0U);
    first = HAL_TIM_PWM_Stop(motor->timer, motor->in1_channel);
    second = HAL_TIM_PWM_Stop(motor->timer, motor->in2_channel);
    motor->initialized = 0U;
    motor->timer = NULL;
    return first != HAL_OK ? first : second;
}

static HAL_StatusTypeDef configure_speed(Motor_HandleTypeDef *motor, uint32_t cpr,
                                         int8_t sign, const PID_Config *config,
                                         Motor_SpeedUnit unit)
{
    PID_Controller candidate = {0};
    Motor_SpeedControl *control = speed_control(motor);
    Encoder_State *encoder;
    if (!systemReady || !motor_valid(motor) || control == NULL ||
        (unit == MOTOR_SPEED_RPM && cpr == 0U) ||
        (sign != 1 && sign != -1) || config == NULL ||
        config->output_min != 0.0f || config->output_max > 1000.0f ||
        (config->kp == 0.0f && config->ki == 0.0f) || !PID_Init(&candidate, config))
        return HAL_ERROR;
    if (Motor_Stop(motor) != HAL_OK) return HAL_ERROR;
    encoder = motor == &motorA ? &encoderA : &encoderB;
    encoder->counts_per_turn = cpr;
    encoder->sign = sign;
    encoder->valid = 0U;
    encoder->rpm = 0.0f;
    control->pid = candidate;
    control->cpr = cpr;
    control->sign = sign;
    control->unit = unit;
    control->feedback_timeout_ms = unit == MOTOR_SPEED_COUNTS ? 300U : 0U;
    control->configured = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Motor_PIDConfigure(Motor_HandleTypeDef *motor, uint32_t cpr,
                                     int8_t sign, const PID_Config *config)
{
    return configure_speed(motor, cpr, sign, config, MOTOR_SPEED_RPM);
}

HAL_StatusTypeDef Motor_PIDConfigureCounts(Motor_HandleTypeDef *motor,
                                           int8_t sign, const PID_Config *config)
{
    return configure_speed(motor, 0U, sign, config, MOTOR_SPEED_COUNTS);
}

HAL_StatusTypeDef Motor_SetFeedbackTimeout(Motor_HandleTypeDef *motor,
                                          uint32_t timeout_ms)
{
    Motor_SpeedControl *control = speed_control(motor);
    if (!motor_valid(motor) || control == NULL || !control->configured ||
        timeout_ms > 60000U || (timeout_ms != 0U && timeout_ms < 10U))
        return HAL_ERROR;
    if (control->enabled) return HAL_BUSY;
    control->feedback_timeout_ms = timeout_ms;
    control->no_feedback_ms = 0U;
    return HAL_OK;
}

static int target_valid(Motor_HandleTypeDef *motor, float target, Motor_SpeedUnit unit)
{
    Motor_SpeedControl *control = speed_control(motor);
    Encoder_State *encoder;
    if (!systemReady || !motor_valid(motor) || control == NULL ||
        target != target || target > FLT_MAX || target < -FLT_MAX) return 0;
    if (target == 0.0f) return 1;
    encoder = motor == &motorA ? &encoderA : &encoderB;
    if (!control->configured || control->unit != unit ||
        encoder->counts_per_turn != control->cpr ||
        encoder->sign != control->sign) return 0;
    if (control->enabled && ((target > 0.0f) != (control->target > 0.0f)))
        return 0;
    return 1;
}

HAL_StatusTypeDef Motor_SetTargetsRPM(float cn1_rpm, float cn2_rpm)
{
    /* Validate BOTH requests before changing either target. Foreground only. */
    if (!target_valid(&motorA, cn1_rpm, MOTOR_SPEED_RPM) ||
        !target_valid(&motorB, cn2_rpm, MOTOR_SPEED_RPM))
        return HAL_ERROR;
    if (Motor_SetTargetRPM(&motorA, cn1_rpm) != HAL_OK ||
        Motor_SetTargetRPM(&motorB, cn2_rpm) != HAL_OK)
    {
        (void)Motor_Stop(&motorA);
        (void)Motor_Stop(&motorB);
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef set_target(Motor_HandleTypeDef *motor, float target,
                                    Motor_SpeedUnit unit)
{
    Motor_SpeedControl *control = speed_control(motor);
    if (!target_valid(motor, target, unit)) return HAL_ERROR;
    if (target == 0.0f) return Motor_Stop(motor);
    if (!control->enabled)
    {
        if (Motor_Stop(motor) != HAL_OK) return HAL_ERROR;
        /* The first control output is computed at the next encoder sample. */
    }
    control->target = target;
    control->enabled = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetTargetRPM(Motor_HandleTypeDef *motor, float rpm)
{
    return set_target(motor, rpm, MOTOR_SPEED_RPM);
}

HAL_StatusTypeDef Motor_SetTargetCountsPerSecond(Motor_HandleTypeDef *motor,
                                                float counts_per_second)
{
    return set_target(motor, counts_per_second, MOTOR_SPEED_COUNTS);
}

HAL_StatusTypeDef Motor_SetTargetsCountsPerSecond(float cn1, float cn2)
{
    if (!target_valid(&motorA, cn1, MOTOR_SPEED_COUNTS) ||
        !target_valid(&motorB, cn2, MOTOR_SPEED_COUNTS)) return HAL_ERROR;
    if (Motor_SetTargetCountsPerSecond(&motorA, cn1) != HAL_OK ||
        Motor_SetTargetCountsPerSecond(&motorB, cn2) != HAL_OK)
    {
        (void)Motor_Stop(&motorA);
        (void)Motor_Stop(&motorB);
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef CSGO(float cn1_speed_percent, float cn2_speed_percent)
{
    if (cn1_speed_percent != cn1_speed_percent || cn2_speed_percent != cn2_speed_percent ||
        cn1_speed_percent < 0.0f || cn1_speed_percent > 100.0f ||
        cn2_speed_percent < 0.0f || cn2_speed_percent > 100.0f)
        return HAL_ERROR;
    return Motor_SetTargetsCountsPerSecond(
        cn1_speed_percent * Motor_FullScaleCountsPerSecond[0] / 100.0f,
        cn2_speed_percent * Motor_FullScaleCountsPerSecond[1] / 100.0f);
}

HAL_StatusTypeDef Motor_PIDDisable(Motor_HandleTypeDef *motor)
{
    if (speed_control(motor) == NULL) return HAL_ERROR;
    return Motor_Stop(motor);
}

uint8_t Motor_PIDIsEnabled(const Motor_HandleTypeDef *motor)
{
    Motor_SpeedControl *control = speed_control(motor);
    return control != NULL ? control->enabled : 0U;
}

static HAL_StatusTypeDef speed_update(uint32_t elapsed_ms)
{
    Motor_HandleTypeDef *motors[2] = {&motorA, &motorB};
    Encoder_State *encoders[2] = {&encoderA, &encoderB};
    int32_t duties[2] = {0, 0};
    float dt = (float)elapsed_ms * 0.001f;
    HAL_StatusTypeDef status = HAL_ERROR;
    unsigned int i;
    for (i = 0U; i < 2U; ++i)
    {
        Motor_SpeedControl *control = &speedControl[i];
        Encoder_State *encoder = encoders[i];
        float direction, output, measurement;
        if (!control->enabled) continue;
        if (!encoder->valid || encoder->counts_per_turn != control->cpr ||
            encoder->sign != control->sign ||
            (control->unit == MOTOR_SPEED_RPM && control->cpr == 0U)) goto fault;
        if (control->feedback_timeout_ms != 0U && control->pid.output >= 0.5f &&
            encoder->delta == 0)
        {
            control->no_feedback_ms += elapsed_ms;
            if (control->no_feedback_ms >= control->feedback_timeout_ms)
            {
                status = HAL_TIMEOUT;
                goto fault;
            }
        }
        else control->no_feedback_ms = 0U;
        direction = control->target > 0.0f ? 1.0f : -1.0f;
        measurement = control->unit == MOTOR_SPEED_COUNTS ?
                      encoder->counts_per_second : encoder->rpm;
        if (!PID_Update(&control->pid, control->target * direction,
                        measurement * direction, dt, &output)) goto fault;
        duties[i] = (int32_t)(output + 0.5f);
        if (direction < 0.0f) duties[i] = -duties[i];
    }
    for (i = 0U; i < 2U; ++i)
        if (speedControl[i].enabled && apply_duty(motors[i], duties[i]) != HAL_OK)
            goto fault;
    return HAL_OK;
fault:
    for (i = 0U; i < 2U; ++i)
        if (speedControl[i].enabled) (void)Motor_Stop(motors[i]);
    return status;
}

#include <limits.h>
//霍尔计数和转速计算
void Encoder_Reset(Encoder_State *s, uint16_t counter)
{
    s->previous = counter;
    s->sign = 1;
    s->counts_per_turn = 0U;
    s->delta = 0;
    s->position = 0;
    s->counts_per_second = 0.0f;
    s->rpm = 0.0f;
    s->valid = 0U;
}

int Encoder_Sample(Encoder_State *s, uint16_t counter, uint32_t elapsed_ms)
{
    uint32_t difference;
    int32_t delta;
    if (s == 0 || elapsed_ms == 0U) return 0;
    difference = (uint16_t)(counter - s->previous);
    s->previous = counter;
    s->valid = 0U;
    s->delta = 0;
    s->counts_per_second = 0.0f;
    s->rpm = 0.0f;
    if (elapsed_ms > 100U || difference == 32768U) return 0;
    delta = difference < 32768U ? (int32_t)difference : (int32_t)difference - 65536;
    if (s->sign < 0) delta = -delta;
    if ((delta > 0 && s->position > INT64_MAX - delta) ||
        (delta < 0 && s->position < INT64_MIN - delta)) return 0;
    s->delta = delta;
    s->position += delta;
    s->counts_per_second = (float)delta * 1000.0f / (float)elapsed_ms;
    if (s->counts_per_turn != 0U)
        s->rpm = s->counts_per_second * 60.0f / (float)s->counts_per_turn;
    s->valid = 1U;
    return 1;
}
