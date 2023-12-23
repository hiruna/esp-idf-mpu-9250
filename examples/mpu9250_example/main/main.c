#include <stdio.h>

#include "esp_log.h"
#include "mpu9250.h"
#include "driver_mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define TAG "mpu9250_example"

static mpu9250_handle_t mpu9250_hdl;

void app_main() {

    i2c_master_bus_config_t i2c_master_conf = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_44,
            .scl_io_num = GPIO_NUM_43,
            .flags.enable_internal_pullup = true,
            .glitch_ignore_cnt = 9,
    };
    i2c_device_config_t i2c_dev_conf = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x68,
            .scl_speed_hz = 400000,
    };
//
//    esp_err_t ret;
//    i2c_master_bus_handle_t i2c_master_bus_hdl;
//    i2c_master_dev_handle_t i2c_master_dev_hdl;
//    vTaskDelay(pdMS_TO_TICKS(1000));
//    ret = i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus_hdl);
//    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "i2c_new_master_bus error!");
//    }
//    ESP_LOGI(TAG, "i2c init ok...");
//    vTaskDelay(pdMS_TO_TICKS(100));
//    ESP_LOGI(TAG, "i2c probe 0x68...");
//    ESP_ERROR_CHECK(i2c_master_probe(i2c_master_bus_hdl, 0x68, 5000));
//    ESP_LOGI(TAG, "i2c probe 0x0C...");
//    ESP_ERROR_CHECK(i2c_master_probe(i2c_master_bus_hdl, 0x0C, 5000));

//    ret = i2c_master_bus_add_device(i2c_master_bus_hdl, &i2c_dev_conf, &i2c_master_dev_hdl);
//    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "i2c_master_bus_add_device error!");
//    }
//    vTaskDelay(pdMS_TO_TICKS(100));
//    uint8_t data[2];
//    uint8_t *wbuf = (uint8_t*)malloc(sizeof(uint8_t));
//    uint8_t reg = 0x75;
//    uint8_t addr = 0x68;
//    //wbuf[0] = 0x68 << 1 | I2C_MASTER_WRITE;
//    //wbuf[1] = reg;
////    wbuf[0] = (addr & (reg << ((2 - 1 - 0) * 8))) >> ((2 - 1 - 0) * 8);
////    wbuf[1] = (addr & (reg << ((2 - 1 - 1) * 8))) >> ((2 - 1 - 1) * 8);
////    wbuf[0] = ((addr << 1) | I2C_MASTER_WRITE);
////    wbuf[1] = reg;
////    wbuf[2] = ((addr << 1) | I2C_MASTER_READ);
////    wbuf[0] = (addr & I2C_MASTER_WRITE);
////    wbuf[1] = reg;
////    wbuf[2] = (addr & I2C_MASTER_READ);;
//  //   wbuf[0] = (addr & (reg << ((2 - 1 - 0) * 8))) >> ((2 - 1 - 0) * 8);
//    // const unsigned char * regg = (const unsigned char *) 0x75;
//    ret = i2c_master_transmit_receive(i2c_master_dev_hdl, &reg, 1, data, 1, 5000);
//    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "i2c_master_transmit_receive error!");
//    }
//    ESP_LOGI(TAG, "WHO_AM_I = [ %X ] >>1 ---> %X",  data[0], data[0] >> 1);
////
    ESP_ERROR_CHECK(mpu9250_init_i2c(&mpu9250_hdl, &i2c_master_conf, &i2c_dev_conf));
    ESP_LOGI(TAG, "basic test...");
    ESP_ERROR_CHECK(mpu9250_basic_read_test(&mpu9250_hdl));


    while (true) {
       // ESP_LOGI(TAG,"waiting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}