#include "motor_command.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>

static Motor_CommandSend send_reply, send_status;
static char sid[33], line[MOTOR_COMMAND_REPLY_SIZE];
static char last_body[MOTOR_COMMAND_REPLY_SIZE], last_reply[MOTOR_COMMAND_REPLY_SIZE];
static char fault[32];
static uint32_t last_byte_tick, last_command_tick, last_status_tick, last_seq;
static uint16_t length, targets[2];
static uint8_t discard, cr_seen, bound, armed, moving, healthy, fault_reported;
static uint8_t start_allowed, button_seen, button_candidate, button_stable, button_released, frame_start_allowed;
static uint32_t button_changed_tick, button_poll_tick;

uint16_t Motor_CommandCRC(const uint8_t *bytes, uint32_t size)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    unsigned int bit;
    for (i = 0U; i < size; ++i)
    {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (bit = 0U; bit < 8U; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

static const char *state(void)
{
    if (fault[0] != '\0') return "FAULT";
    if (!bound) return "UNBOUND";
    if (!armed) return "BOUND";
    if (!start_allowed) return "WAIT_START";
    return moving ? "MOVING" : "IDLE";
}

static int stop(void)
{
    int ok = CSGO(0.0f, 0.0f) == HAL_OK;
    if (!ok)
    {
        (void)Motor_Stop(&motorA);
        (void)Motor_Stop(&motorB);
    }
    targets[0] = targets[1] = 0U;
    moving = 0U;
    return ok;
}

static void failed_reply(void)
{
    (void)stop();
    armed = 0U;
    start_allowed = button_released = 0U;
    (void)snprintf(fault, sizeof(fault), "TX_BACKPRESSURE");
    last_reply[0] = '\0';
    fault_reported = 0U;
}

static int frame(char *out, const char *type, uint32_t seq, const char *payload)
{
    int n = snprintf(out, MOTOR_COMMAND_REPLY_SIZE, "@2,%s,%s,%lu,%s", type, sid, (unsigned long)seq, payload);
    uint16_t crc;
    if (n < 0 || (uint32_t)n + 6U > MOTOR_COMMAND_FRAME_MAX) return 0;
    crc = Motor_CommandCRC((const uint8_t *)out + 1, (uint32_t)n - 1U);
    (void)snprintf(out + n, MOTOR_COMMAND_REPLY_SIZE - (uint32_t)n, "*%04X\n", (unsigned int)crc);
    return 1;
}

static void emit_fault(const char *reason, uint32_t seq)
{
    char message[MOTOR_COMMAND_REPLY_SIZE], payload[48];
    uint8_t changed;
    if (!stop()) reason = "CONTROL";
    changed = (uint8_t)(strcmp(fault, reason) != 0);
    armed = 0U;
    start_allowed = button_released = 0U;
    (void)snprintf(fault, sizeof(fault), "%s", reason);
    last_reply[0] = '\0';
    if (!changed && fault_reported) return;
    (void)snprintf(payload, sizeof(payload), "%s,FAULT", fault);
    if (!frame(message, "ERR", seq, payload) || send_reply == NULL || !send_reply(message))
        failed_reply();
    else fault_reported = 1U;
}

void Motor_CommandFault(const char *reason)
{
    uint8_t broken_frame = (uint8_t)(length != 0U || cr_seen || discard ||
        strcmp(reason, "UART_RX") == 0 || strcmp(reason, "STALE_RX") == 0);
    emit_fault(reason, last_seq);
    length = 0U; cr_seen = 0U; discard = broken_frame;
}

void Motor_CommandSetHealthy(uint8_t value) { healthy = value; }

void Motor_CommandButtonPoll(uint8_t pressed, uint32_t tick)
{
    pressed = (uint8_t)(pressed != 0U);
    /* A held key at boot/recovery, or a gap in sampling, is not a new press. */
    if (!button_seen || (uint32_t)(tick - button_poll_tick) >= MOTOR_COMMAND_PARTIAL_MS)
    {
        button_seen = 1U;
        button_candidate = pressed;
        button_stable = 1U;
        button_released = 0U;
        button_changed_tick = tick;
    }
    button_poll_tick = tick;
    if (pressed != button_candidate)
    {
        button_candidate = pressed;
        button_changed_tick = tick;
    }
    if ((uint32_t)(tick - button_changed_tick) < MOTOR_COMMAND_BUTTON_MS) return;
    if (!pressed)
    {
        button_stable = 0U;
        button_released = 1U;
    }
    else if (!button_stable)
    {
        button_stable = 1U;
        if (button_released && bound && armed && healthy && fault[0] == '\0' && !start_allowed)
        {
            start_allowed = 1U;
        }
        button_released = 0U;
    }
}

static void info(char *message, const char *type)
{
    char payload[96];
    (void)snprintf(payload, sizeof(payload), "%s,%lu,%lu,%lu,%u,%u",
        MOTOR_COMMAND_FIRMWARE, (unsigned long)Motor_FullScaleCountsPerSecond[0],
        (unsigned long)Motor_FullScaleCountsPerSecond[1],
        (unsigned long)Motor_MeasuredCountsPID.output_max,
        (unsigned int)MOTOR_COMMAND_LEASE_MS, (unsigned int)MOTOR_COMMAND_PARTIAL_MS);
    if (!frame(message, type, 0U, payload)) message[0] = '\0';
}

void Motor_CommandInit(Motor_CommandSend send, Motor_CommandSend status_send)
{
    char message[MOTOR_COMMAND_REPLY_SIZE];
    send_reply = send; send_status = status_send;
    (void)snprintf(sid, sizeof(sid), "00000000000000000000000000000000");
    length = discard = cr_seen = bound = armed = moving = 0U;
    start_allowed = button_seen = button_candidate = button_stable = button_released = 0U;
    button_changed_tick = button_poll_tick = 0U;
    frame_start_allowed = 0U;
    healthy = 1U; fault_reported = 0U;
    last_byte_tick = last_command_tick = last_status_tick = last_seq = 0U;
    fault[0] = last_body[0] = last_reply[0] = '\0';
    if (!stop()) { emit_fault("CONTROL", 0U); return; }
    info(message, "READY");
    if (message[0] == '\0' || send_reply == NULL || !send_reply(message)) failed_reply();
}

static int number(const char *text, uint32_t max, uint32_t *out)
{
    uint32_t n = 0U;
    if (*text == '\0' || strlen(text) > 10U) return 0;
    for (; *text; ++text)
    {
        uint32_t digit;
        if (*text < '0' || *text > '9') return 0;
        digit = (uint32_t)(*text - '0');
        if (digit > max || n > (max - digit) / 10U) return 0;
        n = n * 10U + digit;
    }
    *out = n;
    return 1;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int valid_sid(const char *text)
{
    unsigned int i;
    int nonzero = 0;
    if (strlen(text) != 32U) return 0;
    for (i = 0U; i < 32U; ++i)
    {
        int digit = hex_digit(text[i]);
        if (digit < 0) return 0;
        nonzero |= digit;
    }
    return nonzero != 0;
}

static void execute(uint32_t tick)
{
    char original[MOTOR_COMMAND_REPLY_SIZE], payload[48];
    char *fields[7], *cursor;
    unsigned int i, count = 0U;
    uint32_t seq = 0U, a = 0U, b = 0U;
    uint16_t expected = 0U;
    size_t size = strlen(line);
    if (size < 8U || line[0] != '@' || line[size - 5U] != '*')
    { emit_fault("FRAME", 0U); return; }
    for (i = 0U; i < 4U; ++i)
    {
        int digit = hex_digit(line[size - 4U + i]);
        if (digit < 0) { emit_fault("CRC", 0U); return; }
        expected = (uint16_t)((uint32_t)expected * 16U + (uint32_t)digit);
    }
    if (Motor_CommandCRC((const uint8_t *)line + 1, (uint32_t)size - 6U) != expected)
    { emit_fault("CRC", 0U); return; }
    line[size - 5U] = '\0';
    (void)snprintf(original, sizeof(original), "%s", line + 1);
    cursor = line + 1; fields[count++] = cursor;
    while (*cursor)
    {
        if (*cursor == ',')
        {
            *cursor = '\0';
            if (count == 7U) { emit_fault("FIELDS", 0U); return; }
            fields[count++] = cursor + 1;
        }
        ++cursor;
    }
    if (count < 4U || strcmp(fields[0], "2") != 0 || !valid_sid(fields[2]) ||
        !number(fields[3], UINT32_MAX, &seq))
    { emit_fault("FIELDS", 0U); return; }
    if (strcmp(fields[1], "HELLO") == 0)
    {
        if (count != 4U || seq != 0U) { emit_fault("FIELDS", seq); return; }
        if (bound && strcmp(sid, fields[2]) == 0)
        {
            if (last_seq == 0U && strcmp(last_body, original) == 0 && last_reply[0] != '\0')
            { if (!send_reply(last_reply)) failed_reply(); return; }
            emit_fault("STALE_SEQ", seq); return;
        }
        if (!stop()) { emit_fault("CONTROL", seq); return; }
        (void)snprintf(sid, sizeof(sid), "%s", fields[2]);
        bound = 1U; armed = 0U; last_seq = 0U;
        start_allowed = button_released = 0U;
        info(last_reply, "INFO");
    }
    else
    {
        if (!bound || strcmp(sid, fields[2]) != 0) { emit_fault("SESSION", seq); return; }
        if (seq == last_seq && last_reply[0] != '\0' && strcmp(original, last_body) == 0)
        { if (!send_reply(last_reply)) failed_reply(); return; }
        if (seq == 0U || seq <= last_seq) { emit_fault("STALE_SEQ", seq); return; }
        if (strcmp(fields[1], "STOP") == 0)
        {
            if (count != 4U) { emit_fault("FIELDS", seq); return; }
            if (!stop() || !healthy) { emit_fault("NOT_HEALTHY", seq); return; }
            fault[0] = '\0'; armed = 1U;
        }
        else if (strcmp(fields[1], "SET") == 0)
        {
            if (count != 6U || !number(fields[4], 1000U, &a) || !number(fields[5], 1000U, &b))
            { emit_fault("RANGE", seq); return; }
            if (!armed || fault[0] != '\0') { emit_fault("NOT_ARMED", seq); return; }
            /* Do not execute a motion frame started before physical authorization. */
            if ((a != 0U || b != 0U) && (!start_allowed || !frame_start_allowed))
            { emit_fault("START_REQUIRED", seq); return; }
            if (!healthy || CSGO((float)a / 10.0f, (float)b / 10.0f) != HAL_OK)
            { emit_fault("CONTROL", seq); return; }
            targets[0] = (uint16_t)a; targets[1] = (uint16_t)b;
            moving = (uint8_t)(a != 0U || b != 0U);
            last_command_tick = tick;
        }
        else { emit_fault("TYPE", seq); return; }
        last_seq = seq;
        (void)snprintf(payload, sizeof(payload), "%s,%s,%u,%u", fields[1], state(),
            (unsigned int)targets[0], (unsigned int)targets[1]);
        if (!frame(last_reply, "ACK", seq, payload)) { failed_reply(); return; }
    }
    (void)snprintf(last_body, sizeof(last_body), "%s", original);
    if (last_reply[0] == '\0' || send_reply == NULL || !send_reply(last_reply)) failed_reply();
}

void Motor_CommandPoll(uint32_t tick)
{
    if (moving && (uint32_t)(tick - last_command_tick) >= MOTOR_COMMAND_LEASE_MS)
        Motor_CommandFault("LINK_TIMEOUT");
    if (length != 0U && (uint32_t)(tick - last_byte_tick) >= MOTOR_COMMAND_PARTIAL_MS)
        Motor_CommandFault("PARTIAL_TIMEOUT");
    if ((uint32_t)(tick - last_status_tick) >= MOTOR_COMMAND_STATUS_MS)
    {
        char message[MOTOR_COMMAND_REPLY_SIZE], payload[128];
        Motor_FeedbackSnapshot snapshot;
        last_status_tick = tick;
        Motor_GetFeedbackSnapshot(&snapshot);
        (void)snprintf(payload, sizeof(payload), "%s,%u,%u,%ld,%ld,%u,%lu,%s,%lu", state(),
            (unsigned int)targets[0], (unsigned int)targets[1], (long)snapshot.cps10[0],
            (long)snapshot.cps10[1], (unsigned int)snapshot.valid_bits,
            (unsigned long)snapshot.age_ms, fault[0] ? fault : "NONE", (unsigned long)tick);
        if (frame(message, "STATUS", last_seq, payload) && send_status != NULL)
            (void)send_status(message);
    }
}

void Motor_CommandReceive(uint8_t byte, uint32_t tick)
{
    last_byte_tick = tick;
    if (byte == '\n')
    {
        if (!discard && length != 0U) { line[length] = '\0'; execute(tick); }
        length = 0U; discard = cr_seen = 0U;
        return;
    }
    if (discard) return;
    if (byte == '\r')
    {
        if (cr_seen || length + 2U > MOTOR_COMMAND_FRAME_MAX) Motor_CommandFault("FRAME");
        else cr_seen = 1U;
        return;
    }
    if (cr_seen || byte < 33U || byte > 126U || length + 1U >= MOTOR_COMMAND_FRAME_MAX)
    { Motor_CommandFault("FRAME"); discard = 1U; return; }
    if (length == 0U) frame_start_allowed = start_allowed;
    line[length++] = (char)byte;
}
