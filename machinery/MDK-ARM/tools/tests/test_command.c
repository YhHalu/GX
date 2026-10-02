#include "motor_command.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float target_first, target_second;
static unsigned int apply_count, replies;
static int fail_apply, fail_send;
static char response[MOTOR_COMMAND_REPLY_SIZE], telemetry[MOTOR_COMMAND_REPLY_SIZE];
static const char *session = "11111111111111111111111111111111";
Motor_HandleTypeDef motorA, motorB;
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor)
{ if (motor==&motorA) target_first=0; else target_second=0; return HAL_OK; }
HAL_StatusTypeDef CSGO(float a, float b)
{
    if (fail_apply) return HAL_ERROR;
    target_first=a; target_second=b; ++apply_count;
    return HAL_OK;
}
void Motor_GetFeedbackSnapshot(Motor_FeedbackSnapshot *s)
{ s->cps10[0]=12345; s->cps10[1]=-100; s->valid_bits=3; s->age_ms=5; }
static int capture(const char *message)
{
    if (fail_send) return 0;
    assert(strlen(message) <= MOTOR_COMMAND_FRAME_MAX);
    strcpy(response,message); ++replies;
    return 1;
}
static int status_capture(const char *message) { strcpy(telemetry,message); return 0; }
static void feed(const char *bytes,uint32_t tick)
{ while (*bytes) Motor_CommandReceive((uint8_t)*bytes++,tick); }
static void request(const char *type,const char *sid,uint32_t seq,const char *args,uint32_t tick)
{
    char data[256];
    int n=snprintf(data,sizeof(data),"@2,%s,%s,%lu%s%s",type,sid,(unsigned long)seq,args[0]?",":"",args);
    snprintf(data+n,sizeof(data)-(size_t)n,"*%04X\n",(unsigned int)Motor_CommandCRC((uint8_t*)data+1,(uint32_t)n-1U));
    feed(data,tick);
}
static void handshake(uint32_t tick)
{
    Motor_CommandInit(capture,status_capture);
    request("HELLO",session,0,"",tick-70U);
    assert(strstr(response,",INFO,")!=NULL);
    request("STOP",session,1,"",tick-69U);
    assert(strstr(response,",STOP,WAIT_START,0,0*")!=NULL);
    Motor_CommandButtonPoll(0,tick-62U); Motor_CommandButtonPoll(0,tick-32U);
    Motor_CommandButtonPoll(1,tick-30U); Motor_CommandButtonPoll(1,tick);
}
static void press(uint32_t tick)
{
    Motor_CommandButtonPoll(0,tick-62U); Motor_CommandButtonPoll(0,tick-32U);
    Motor_CommandButtonPoll(1,tick-30U); Motor_CommandButtonPoll(1,tick);
}
int main(void)
{
    unsigned int before, count;
    assert(Motor_CommandCRC((uint8_t*)"123456789",9)==0x29B1);
    handshake(1);
    request("SET",session,2,"303,600",10);
    assert(fabsf(target_first-30.3f)<0.001f && target_second==60.0f);
    before=apply_count;
    request("SET",session,2,"303,600",100);
    assert(apply_count==before); /* Duplicate does not execute or renew lease. */
    Motor_CommandPoll(509); assert(target_first>0);
    Motor_CommandPoll(510); assert(target_first==0 && strstr(response,"LINK_TIMEOUT"));
    request("SET",session,3,"1000,0",511); assert(target_first==0 && strstr(response,"NOT_ARMED"));
    request("STOP",session,4,"",512);
    press(512);
    request("SET",session,5,"1000,0",513); assert(target_first==100);
    request("SET",session,5,"0,1000",514); assert(target_first==0 && strstr(response,"STALE_SEQ"));
    request("STOP",session,6,"",515);
    request("SET",session,4,"1000,0",516); assert(target_first==0);
    request("STOP",session,7,"",517);
    request("SET","22222222222222222222222222222222",8,"1000,0",518);
    assert(target_first==0 && strstr(response,"SESSION"));
    handshake(20);
    request("SET",session,2,"1001,0",30); assert(target_first==0 && strstr(response,"RANGE"));
    request("STOP",session,3,"",31);
    feed("@2,SET,11111111111111111111111111111111,4,1000,0*0000\n",32);
    assert(target_first==0 && strstr(response,"CRC"));
    request("STOP",session,5,"",33);
    feed("@2,",40); Motor_CommandPoll(139); assert(strstr(response,"PARTIAL_TIMEOUT")==NULL);
    Motor_CommandPoll(140); assert(strstr(response,"PARTIAL_TIMEOUT"));
    feed("\n",141); request("STOP",session,6,"",142);
    feed("3030\n",143); assert(strstr(response,"FRAME"));
    feed("\n",144); request("STOP",session,7,"",145);
    before=replies;
    for(count=0;count<300;count++) Motor_CommandReceive('A',146);
    assert(replies==before+1); feed("\n",147);
    request("STOP",session,8,"",148);
    Motor_CommandSetHealthy(0); request("STOP",session,9,"",149);
    assert(strstr(response,"NOT_HEALTHY"));
    Motor_CommandSetHealthy(1); request("STOP",session,10,"",150);
    press(150);
    Motor_CommandPoll(250); assert(strstr(telemetry,"IDLE,0,0,12345,-100,3,5,NONE,250"));
    assert(target_first==0); /* Telemetry rejection does not latch. */
    fail_send=1; request("SET",session,11,"1000,0",251); assert(target_first==0);
    fail_send=0; request("SET",session,12,"1000,0",252); assert(target_first==0);
    handshake(UINT32_MAX-200U);
    request("SET",session,2,"1000,0",UINT32_MAX-100U);
    Motor_CommandPoll(398); assert(target_first==100);
    Motor_CommandPoll(399); assert(target_first==0);
    handshake(1); request("SET",session,UINT32_MAX,"1000,0",10);
    request("STOP",session,1,"",11); assert(target_first==0 && strstr(response,"STALE_SEQ"));
    handshake(1); fail_apply=1; request("SET",session,2,"1000,0",10);
    assert(strstr(response,"CONTROL")); fail_apply=0;
    Motor_CommandInit(capture,status_capture);
    request("SET",session,2,"1000,0",10); assert(target_first==0 && strstr(response,"SESSION"));

    /* Startup gate: handshake and zero commands cannot substitute for SW2. */
    Motor_CommandInit(capture,status_capture);
    Motor_CommandButtonPoll(1,0); Motor_CommandButtonPoll(1,30);
    request("HELLO",session,0,"",31); request("STOP",session,1,"",32);
    assert(strstr(response,"STOP,WAIT_START,0,0"));
    Motor_CommandButtonPoll(1,60);
    request("SET",session,2,"0,0",61); assert(strstr(response,"SET,WAIT_START,0,0"));
    request("SET",session,3,"100,0",62); assert(target_first==0 && strstr(response,"START_REQUIRED"));
    request("STOP",session,4,"",63);
    Motor_CommandButtonPoll(1,90); /* Holding through fault recovery cannot start. */
    Motor_CommandPoll(100); assert(strstr(telemetry,"WAIT_START"));
    Motor_CommandButtonPoll(0,101); Motor_CommandButtonPoll(0,131);
    Motor_CommandButtonPoll(1,132); Motor_CommandButtonPoll(0,140);
    Motor_CommandButtonPoll(1,141); Motor_CommandButtonPoll(1,170);
    Motor_CommandPoll(170); assert(strstr(telemetry,"WAIT_START"));
    Motor_CommandButtonPoll(1,171); Motor_CommandPoll(200);
    assert(strstr(telemetry,"IDLE,0,0")); assert(target_first==0);
    request("SET",session,5,"100,0",301); assert(target_first==10);
    request("STOP",session,6,"",302); assert(strstr(response,"STOP,IDLE,0,0"));
    request("SET",session,7,"100,0",303); assert(target_first==10);
    request("HELLO","22222222222222222222222222222222",0,"",304);
    request("STOP","22222222222222222222222222222222",1,"",305);
    assert(strstr(response,"STOP,WAIT_START,0,0") && target_first==0);

    /* Partial motion that started before the key must not be replayed afterwards. */
    Motor_CommandInit(capture,status_capture);
    request("HELLO",session,0,"",0); request("STOP",session,1,"",1);
    Motor_CommandButtonPoll(0,2); Motor_CommandButtonPoll(0,32);
    feed("@2,",33);
    Motor_CommandButtonPoll(1,34); Motor_CommandButtonPoll(1,64);
    { char data[200]; int n=snprintf(data,sizeof(data),"2,SET,%s,2,100,0",session);
      snprintf(data+n,sizeof(data)-(size_t)n,"*%04X\n",(unsigned int)Motor_CommandCRC((uint8_t*)data,(uint32_t)n));
      feed(data+2,65); }
    assert(target_first==0 && strstr(response,"START_REQUIRED"));
    /* An unhealthy press does not clear faults or grant permission. */
    request("STOP",session,3,"",66); Motor_CommandSetHealthy(0); press(130);
    Motor_CommandSetHealthy(1); Motor_CommandPoll(200); assert(strstr(telemetry,"WAIT_START"));
    Motor_CommandButtonPoll(1,201); Motor_CommandPoll(300); assert(strstr(telemetry,"WAIT_START"));
    press(365); request("SET",session,4,"100,0",366); assert(target_first==10);
    puts("PASS: CRC, sessions, handshake, replay, lease, range, partial/oversize, health, backpressure, telemetry, restart, wrap");
    return 0;
}
