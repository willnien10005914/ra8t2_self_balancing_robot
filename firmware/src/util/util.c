#include "util/util.h"

static uint32_t (*s_time_fn)(void);

void time_ms_set_port(uint32_t (*fn)(void))
{
    s_time_fn = fn;
}

uint32_t time_ms_get(void)
{
    return s_time_fn ? s_time_fn() : 0u;
}

void ringbuf_init(ringbuf_t *rb, uint8_t *storage, size_t cap)
{
    rb->buf = storage;
    rb->cap = cap;
    rb->head = rb->tail = 0;
}

bool ringbuf_push(ringbuf_t *rb, uint8_t b)
{
    size_t next = (rb->head + 1u) % rb->cap;
    if (next == rb->tail) return false;
    rb->buf[rb->head] = b;
    rb->head = next;
    return true;
}

bool ringbuf_pop(ringbuf_t *rb, uint8_t *b)
{
    if (rb->head == rb->tail) return false;
    *b = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1u) % rb->cap;
    return true;
}

size_t ringbuf_count(const ringbuf_t *rb)
{
    if (rb->head >= rb->tail) return rb->head - rb->tail;
    return rb->cap - rb->tail + rb->head;
}

uint8_t crc8_maxim(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
        {
            if (crc & 0x80u) crc = (uint8_t)((crc << 1) ^ 0x07u);
            else crc <<= 1;
        }
    }
    return crc;
}
