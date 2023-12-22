#include <stdio.h>
#include "esp_log.h"
#include "mpu9250.h"
//#include "driver_mpu9250.h"

#include "driver/i2c_master.h"

#define TAG "mpu9250_example"

static mpu9250_handle_t mpu9250_hdl;

void app_main() {
    i2c_master_bus_config_t i2c_master_conf = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_21,
            .scl_io_num = GPIO_NUM_22,
            .flags.enable_internal_pullup = true,
    };
    i2c_device_config_t i2c_dev_conf = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = MPU9250_ADDRESS_AD0_LOW,
            .scl_speed_hz = 400000,
    };
    mpu9250_init_i2c(&mpu9250_hdl, &i2c_master_conf, &i2c_dev_conf);

}