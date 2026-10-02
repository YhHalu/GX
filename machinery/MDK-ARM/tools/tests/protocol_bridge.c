/* Host-only bridge: execute the real C parser from Python tests, no hardware. */
#include "motor_command.h"
#include "motor.h"
static float values[2];
static unsigned int applies;
Motor_HandleTypeDef motorA, motorB;
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor)
{ values[motor==&motorA?0:1]=0; return HAL_OK; }
HAL_StatusTypeDef CSGO(float a,float b) { values[0]=a; values[1]=b; ++applies; return HAL_OK; }
void Motor_GetFeedbackSnapshot(Motor_FeedbackSnapshot *s)
{ s->cps10[0]=0; s->cps10[1]=0; s->valid_bits=3; s->age_ms=0; }
float Bridge_Target(unsigned int i) { return i<2?values[i]:0; }
unsigned int Bridge_Applies(void) { return applies; }
