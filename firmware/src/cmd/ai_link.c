#include "cmd/ai_link.h"
#include "cmd/cmd_arbiter.h"
#include "util/util.h"

#include <string.h>

#define PAYLOAD_MAX 32

typedef enum {
    ST_SYNC0 = 0,
    ST_SYNC1,
    ST_LEN,
    ST_TYPE,
    ST_PAY,
    ST_CRC
} rx_st_t;

static rx_st_t s_st;
static uint8_t s_len;
static uint8_t s_type;
static uint8_t s_pay[PAYLOAD_MAX];
static uint8_t s_idx;
static uint8_t s_crc_acc;

__attribute__((weak)) void ai_uart_write(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
}

static float rd_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

static void wr_f32(uint8_t *p, float f)
{
    memcpy(p, &f, 4);
}

static void handle_frame(void)
{
    motion_cmd_t m;
    memset(&m, 0, sizeof(m));
    m.source = CMD_SRC_AI;
    m.stamp_ms = time_ms_get();

    switch (s_type)
    {
        case AI_TYPE_HEARTBEAT:
            /* refresh timeout without changing vx/wz — inject 0 brake hold via tick stamp */
            cmd_arbiter_get_motion(&m);
            m.source = CMD_SRC_AI;
            m.stamp_ms = time_ms_get();
            m.lqr_valid = false;
            cmd_arbiter_inject_motion(&m);
            break;
        case AI_TYPE_MOTION:
            if (s_len >= 8)
            {
                m.vx_mps = rd_f32(&s_pay[0]);
                m.wz_radps = rd_f32(&s_pay[4]);
                m.lqr_valid = false;
                cmd_arbiter_inject_motion(&m);
            }
            break;
        case AI_TYPE_BRAKE:
        case AI_TYPE_STOP_FOLLOW:
            m.brake = true;
            m.lqr_valid = false;
            cmd_arbiter_inject_motion(&m);
            break;
        case AI_TYPE_LQR:
            if (s_len >= 1)
            {
                m.lqr_valid = true;
                m.lqr_enable = (s_pay[0] != 0);
                m.brake = true;
                cmd_arbiter_inject_motion(&m);
            }
            break;
        case AI_TYPE_ESTOP:
            m.estop = true;
            cmd_arbiter_inject_motion(&m);
            break;
        case AI_TYPE_TELEM_REQ:
            /* caller may periodically push telemetry from app */
            break;
        default:
            break;
    }
}

void ai_link_init(void)
{
    s_st = ST_SYNC0;
}

void ai_link_rx_byte(uint8_t b)
{
    switch (s_st)
    {
        case ST_SYNC0:
            if (b == AI_SYNC0) s_st = ST_SYNC1;
            break;
        case ST_SYNC1:
            s_st = (b == AI_SYNC1) ? ST_LEN : ST_SYNC0;
            break;
        case ST_LEN:
            s_len = b;
            s_crc_acc = b;
            s_idx = 0;
            s_st = ST_TYPE;
            break;
        case ST_TYPE:
            s_type = b;
            if (s_len == 0)
            {
                uint8_t tmp[2] = { s_len, s_type };
                s_crc_acc = crc8_maxim(tmp, 2);
                s_st = ST_CRC;
            }
            else if (s_len > PAYLOAD_MAX)
            {
                s_st = ST_SYNC0;
            }
            else
            {
                s_st = ST_PAY;
            }
            break;
        case ST_PAY:
            s_pay[s_idx++] = b;
            if (s_idx >= s_len)
            {
                uint8_t tmp[2 + PAYLOAD_MAX];
                tmp[0] = s_len;
                tmp[1] = s_type;
                memcpy(&tmp[2], s_pay, s_len);
                s_crc_acc = crc8_maxim(tmp, (size_t)s_len + 2u);
                s_st = ST_CRC;
            }
            break;
        case ST_CRC:
            if (b == s_crc_acc)
            {
                handle_frame();
            }
            s_st = ST_SYNC0;
            break;
    }
}

void ai_link_poll(uint32_t now_ms)
{
    (void)now_ms;
}

void ai_link_send_telemetry(float pitch, float vx, float wz, uint16_t fault)
{
    uint8_t frame[3 + 14 + 1];
    uint8_t pay[14];
    wr_f32(&pay[0], pitch);
    wr_f32(&pay[4], vx);
    wr_f32(&pay[8], wz);
    pay[12] = (uint8_t)(fault & 0xFF);
    pay[13] = (uint8_t)(fault >> 8);

    frame[0] = AI_SYNC0;
    frame[1] = AI_SYNC1;
    frame[2] = 14;
    frame[3] = AI_TYPE_TELEM_RESP;
    memcpy(&frame[4], pay, 14);
    {
        uint8_t tmp[16];
        tmp[0] = 14;
        tmp[1] = AI_TYPE_TELEM_RESP;
        memcpy(&tmp[2], pay, 14);
        frame[4 + 14] = crc8_maxim(tmp, 16);
    }
    ai_uart_write(frame, sizeof(frame));
}
