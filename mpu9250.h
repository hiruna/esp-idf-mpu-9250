#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"


esp_err_t mpu9250_init_i2c(i2c_config_t *i2c_conf, i2c_port_t port);

#ifdef __cplusplus
}
#endif
