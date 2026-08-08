#pragma once

#include "app/app_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Entry from FSP generated hal_entry(). Never returns. */
void app_main(void);

#ifdef __cplusplus
}
#endif
