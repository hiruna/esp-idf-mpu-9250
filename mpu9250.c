#include "stdio.h"
#include "mpu9250.h"
#include "esp_err.h"
#include "driver_mpu9250_interface.h"

static void mpu9250_link_interface_functions(mpu9250_handle_t *mpu9250_hdl) {
    DRIVER_MPU9250_LINK_INIT(mpu9250_hdl, mpu9250_handle_t);
    DRIVER_MPU9250_LINK_IIC_INIT(mpu9250_hdl, mpu9250_interface_iic_init);
    DRIVER_MPU9250_LINK_IIC_DEINIT(mpu9250_hdl, mpu9250_interface_iic_deinit);
    DRIVER_MPU9250_LINK_IIC_READ(mpu9250_hdl, mpu9250_interface_iic_read);
    DRIVER_MPU9250_LINK_IIC_WRITE(mpu9250_hdl, mpu9250_interface_iic_write);
    DRIVER_MPU9250_LINK_SPI_INIT(mpu9250_hdl, mpu9250_interface_spi_init);
    DRIVER_MPU9250_LINK_SPI_DEINIT(mpu9250_hdl, mpu9250_interface_spi_deinit);
    DRIVER_MPU9250_LINK_SPI_READ(mpu9250_hdl, mpu9250_interface_spi_read);
    DRIVER_MPU9250_LINK_SPI_WRITE(mpu9250_hdl, mpu9250_interface_spi_write);
    DRIVER_MPU9250_LINK_DELAY_MS(mpu9250_hdl, mpu9250_interface_delay_ms);
    DRIVER_MPU9250_LINK_DEBUG_PRINT(mpu9250_hdl, mpu9250_interface_debug_print);
    DRIVER_MPU9250_LINK_RECEIVE_CALLBACK(mpu9250_hdl, mpu9250_interface_receive_callback);

}
esp_err_t mpu9250_init_i2c(i2c_config_t *i2c_conf,i2c_port_t port) {
    esp_err_t ret;
    DRIVER_MPU9250_LINK_INIT
    mpu9250_interface_t t;
    ret = mpu9250_set_i2c_config(i2c_conf, port);
    assert(ESP_OK == ret);

    return ret;
}
void hello() {
    printf("hello");
}