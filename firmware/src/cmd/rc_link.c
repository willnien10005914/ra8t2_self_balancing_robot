#include "cmd/rc_link.h"

void rc_link_init(void)
{
}

void rc_link_poll(uint32_t now_ms)
{
    (void)now_ms;
    /* Future: parse CRSF/SBUS and cmd_arbiter_inject_motion(). */
}
