#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t head;
    size_t tail;
} ringbuf_t;

void ringbuf_init(ringbuf_t *rb, uint8_t *storage, size_t cap);
bool ringbuf_push(ringbuf_t *rb, uint8_t b);
bool ringbuf_pop(ringbuf_t *rb, uint8_t *b);
size_t ringbuf_count(const ringbuf_t *rb);

uint8_t crc8_maxim(const uint8_t *data, size_t len);
uint32_t time_ms_get(void);
void time_ms_set_port(uint32_t (*fn)(void));

#ifdef __cplusplus
}
#endif
