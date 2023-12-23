#include "stdio.h"
#include "mpu9250.h"
#include "esp_err.h"

static const char *MPU9250_TAG = "mpu9250";

static void mpu9250_link_bus_interface_functions(mpu9250_handle_t *mpu9250_hdl) {
    DRIVER_MPU9250_LINK_IIC_INIT(mpu9250_hdl, mpu9250_interface_iic_init);
    DRIVER_MPU9250_LINK_IIC_DEINIT(mpu9250_hdl, mpu9250_interface_iic_deinit);
    DRIVER_MPU9250_LINK_IIC_READ(mpu9250_hdl, mpu9250_interface_iic_read);
    DRIVER_MPU9250_LINK_IIC_WRITE(mpu9250_hdl, mpu9250_interface_iic_write);
    DRIVER_MPU9250_LINK_SPI_INIT(mpu9250_hdl, mpu9250_interface_spi_init);
    DRIVER_MPU9250_LINK_SPI_DEINIT(mpu9250_hdl, mpu9250_interface_spi_deinit);
    DRIVER_MPU9250_LINK_SPI_READ(mpu9250_hdl, mpu9250_interface_spi_read);
    DRIVER_MPU9250_LINK_SPI_WRITE(mpu9250_hdl, mpu9250_interface_spi_write);
}

esp_err_t mpu9250_init_i2c(mpu9250_handle_t *mpu9250_hdl, i2c_master_bus_config_t *i2c_master_conf,
                           i2c_device_config_t *i2c_dev_conf) {
    esp_err_t ret;


    ESP_LOGD(MPU9250_TAG, "set i2c conf");
    ret = mpu9250_set_i2c_config(i2c_master_conf, i2c_dev_conf);
    assert(ESP_OK == ret);
    ESP_LOGD(MPU9250_TAG, "mpu iface fn links init");
    DRIVER_MPU9250_LINK_INIT(mpu9250_hdl, mpu9250_handle_t);
    DRIVER_MPU9250_LINK_DELAY_MS(mpu9250_hdl, mpu9250_interface_delay_ms);
    DRIVER_MPU9250_LINK_DEBUG_PRINT(mpu9250_hdl, mpu9250_interface_debug_print);
    DRIVER_MPU9250_LINK_RECEIVE_CALLBACK(mpu9250_hdl, mpu9250_debug_receive_callback);
    ESP_LOGD(MPU9250_TAG, "mpu iface bus fn links");
    mpu9250_link_bus_interface_functions(mpu9250_hdl);

    ESP_LOGD(MPU9250_TAG, "mpu iface set");
    ret = mpu9250_set_interface(mpu9250_hdl, MPU9250_INTERFACE_IIC);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set i2c interface failed!");
        return ESP_FAIL;
    }
    ESP_LOGD(MPU9250_TAG, "mpu i2c addr set");
    ret = mpu9250_set_addr_pin(mpu9250_hdl, (*i2c_dev_conf).device_address);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set i2c address failed!");
        return ESP_FAIL;
    }
    ESP_LOGD(MPU9250_TAG, "mpu init");
    ret = mpu9250_init(mpu9250_hdl);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mpu init failed!");
        return ESP_FAIL;
    }
    ESP_LOGD(MPU9250_TAG, "sleeping 100 ms");
    mpu9250_interface_delay_ms(100);
    ret = mpu9250_set_sleep(mpu9250_hdl, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set sleep to false failed");
        (void) mpu9250_deinit(mpu9250_hdl);
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t mpu9250_basic_read_test(mpu9250_handle_t *mpu9250_hdl) {
    esp_err_t ret;
    /* set fifo 1024kb */
    ret = mpu9250_set_fifo_1024kb(mpu9250_hdl);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo 1024kb failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default clock source */
    ret = mpu9250_set_clock_source(mpu9250_hdl, MPU9250_BASIC_DEFAULT_CLOCK_SOURCE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set clock source failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default rate */
    ret = mpu9250_set_sample_rate_divider(mpu9250_hdl, 1000 / (MPU9250_BASIC_DEFAULT_RATE - 1));
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set sample rate divider failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable temperature sensor */
    ret = mpu9250_set_ptat(mpu9250_hdl, MPU9250_BOOL_TRUE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set ptat failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default cycle wake up */
    ret = mpu9250_set_cycle_wake_up(mpu9250_hdl, MPU9250_BASIC_DEFAULT_CYCLE_WAKE_UP);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set cycle wake up failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable acc x */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_ACC_X, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable acc y */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_ACC_Y, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable acc z */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_ACC_Z, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable gyro x */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_GYRO_X, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable gyro y */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_GYRO_Y, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* enable gyro z */
    ret = mpu9250_set_standby_mode(mpu9250_hdl, MPU9250_SOURCE_GYRO_Z, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set standby mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable gyroscope x test */
    ret = mpu9250_set_gyroscope_test(mpu9250_hdl, MPU9250_AXIS_X, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyroscope test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable gyroscope y test */
    ret = mpu9250_set_gyroscope_test(mpu9250_hdl, MPU9250_AXIS_Y, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyroscope test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable gyroscope z test */
    ret = mpu9250_set_gyroscope_test(mpu9250_hdl, MPU9250_AXIS_Z, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyroscope test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable accelerometer x test */
    ret = mpu9250_set_accelerometer_test(mpu9250_hdl, MPU9250_AXIS_X, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable accelerometer y test */
    ret = mpu9250_set_accelerometer_test(mpu9250_hdl, MPU9250_AXIS_Y, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable accelerometer z test */
    ret = mpu9250_set_accelerometer_test(mpu9250_hdl, MPU9250_AXIS_Z, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer test failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable fifo */
    ret = mpu9250_set_fifo(mpu9250_hdl, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable temp fifo */
    ret = mpu9250_set_fifo_enable(mpu9250_hdl, MPU9250_FIFO_TEMP, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo enable failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable xg fifo */
    ret = mpu9250_set_fifo_enable(mpu9250_hdl, MPU9250_FIFO_XG, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo enable failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable yg fifo */
    ret = mpu9250_set_fifo_enable(mpu9250_hdl, MPU9250_FIFO_YG, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo enable failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable zg fifo */
    ret = mpu9250_set_fifo_enable(mpu9250_hdl, MPU9250_FIFO_ZG, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo enable failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* disable accel fifo */
    ret = mpu9250_set_fifo_enable(mpu9250_hdl, MPU9250_FIFO_ACCEL, MPU9250_BOOL_FALSE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo enable failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default interrupt level */
    ret = mpu9250_set_interrupt_level(mpu9250_hdl, MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_LEVEL);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt level failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default pin type */
    ret = mpu9250_set_interrupt_pin_type(mpu9250_hdl, MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_TYPE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt pin type failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default motion interrupt */
    ret = mpu9250_set_interrupt(mpu9250_hdl, MPU9250_INTERRUPT_MOTION, MPU9250_BASIC_DEFAULT_INTERRUPT_MOTION);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default fifo overflow interrupt */
    ret = mpu9250_set_interrupt(mpu9250_hdl, MPU9250_INTERRUPT_FIFO_OVERFLOW,
                                MPU9250_BASIC_DEFAULT_INTERRUPT_FIFO_OVERFLOW);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default dmp interrupt */
    ret = mpu9250_set_interrupt(mpu9250_hdl, MPU9250_INTERRUPT_DMP, MPU9250_BASIC_DEFAULT_INTERRUPT_DMP);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default fsync int interrupt */
    ret = mpu9250_set_interrupt(mpu9250_hdl, MPU9250_INTERRUPT_FSYNC_INT, MPU9250_BASIC_DEFAULT_INTERRUPT_FSYNC_INT);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default data ready interrupt */
    ret = mpu9250_set_interrupt(mpu9250_hdl, MPU9250_INTERRUPT_DATA_READY, MPU9250_BASIC_DEFAULT_INTERRUPT_DATA_READY);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default interrupt latch */
    ret = mpu9250_set_interrupt_latch(mpu9250_hdl, MPU9250_BASIC_DEFAULT_INTERRUPT_LATCH);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt latch failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default interrupt read clear */
    ret = mpu9250_set_interrupt_read_clear(mpu9250_hdl, MPU9250_BASIC_DEFAULT_INTERRUPT_READ_CLEAR);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set interrupt read clear failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the extern sync */
    ret = mpu9250_set_extern_sync(mpu9250_hdl, MPU9250_BASIC_DEFAULT_EXTERN_SYNC);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set extern sync failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default fsync interrupt */
    ret = mpu9250_set_fsync_interrupt(mpu9250_hdl, MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fsync interrupt failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default fsync interrupt level */
    ret = mpu9250_set_fsync_interrupt_level(mpu9250_hdl, MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT_LEVEL);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fsync interrupt level failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default iic master */
    ret = mpu9250_set_iic_master(mpu9250_hdl, MPU9250_BASIC_DEFAULT_IIC_MASTER);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set iic master failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default iic bypass */
    ret = mpu9250_set_iic_bypass(mpu9250_hdl, MPU9250_BASIC_DEFAULT_IIC_BYPASS);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set iic bypass failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default accelerometer range */
    ret = mpu9250_set_accelerometer_range(mpu9250_hdl, MPU9250_BASIC_DEFAULT_ACCELEROMETER_RANGE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer range failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default gyroscope range */
    ret = mpu9250_set_gyroscope_range(mpu9250_hdl, MPU9250_BASIC_DEFAULT_GYROSCOPE_RANGE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyroscope range failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default gyro standby */
    ret = mpu9250_set_gyro_standby(mpu9250_hdl, MPU9250_BASIC_DEFAULT_GYROSCOPE_STANDBY);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyro standby failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default fifo mode */
    ret = mpu9250_set_fifo_mode(mpu9250_hdl, MPU9250_BASIC_DEFAULT_FIFO_MODE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set fifo mode failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default gyroscope choice */
    ret = mpu9250_set_gyroscope_choice(mpu9250_hdl, MPU9250_BASIC_DEFAULT_GYROSCOPE_CHOICE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set gyroscope choice failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default low pass filter */
    ret = mpu9250_set_low_pass_filter(mpu9250_hdl, MPU9250_BASIC_DEFAULT_LOW_PASS_FILTER);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set low pass filter failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default accelerometer choice */
    ret = mpu9250_set_accelerometer_choice(mpu9250_hdl, MPU9250_BASIC_DEFAULT_ACCELEROMETER_CHOICE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer choice failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default accelerometer low pass filter */
    ret = mpu9250_set_accelerometer_low_pass_filter(mpu9250_hdl, MPU9250_BASIC_DEFAULT_ACCELEROMETER_LOW_PASS_FILTER);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accelerometer low pass filter failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default low power accel output rate */
    ret = mpu9250_set_low_power_accel_output_rate(mpu9250_hdl, MPU9250_BASIC_DEFAULT_LOW_POWER_ACCEL_OUTPUT_RATE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set low power accel output rate failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default wake on motion */
    ret = mpu9250_set_wake_on_motion(mpu9250_hdl, MPU9250_BASIC_DEFAULT_WAKE_ON_MOTION);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set wake on motion failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the default accel compare with previous sample */
    ret = mpu9250_set_accel_compare_with_previous_sample(mpu9250_hdl, MPU9250_BASIC_DEFAULT_ACCELEROMETER_COMPARE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "set accel compare with previous sample failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }



    /* mag init */
    ret = mpu9250_mag_init(mpu9250_hdl);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mag init failed.");
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the mag default mode */
    ret = mpu9250_mag_set_mode(mpu9250_hdl, MPU9250_BASIC_DEFAULT_MAGNETOMETER_MODE);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mag set mode failed.");
        (void) mpu9250_mag_deinit(mpu9250_hdl);
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }

    /* set the mag default bits */
    ret = mpu9250_mag_set_bits(mpu9250_hdl, MPU9250_BASIC_DEFAULT_MAGNETOMETER_BITS);
    if (ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mag set bits failed.");
        (void) mpu9250_mag_deinit(mpu9250_hdl);
        (void) mpu9250_deinit(mpu9250_hdl);

        return ESP_FAIL;
    }


    int MAX_ITERATIONS = 10;
    for(int i=0;i<MAX_ITERATIONS;i++) {
        uint16_t len;
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        int16_t mag_raw[3];
        float accel[3];
        float gyro[3];
        float mag[3];

        /* set 1 */
        len = 1;

        /* read data */
        ret = mpu9250_read(mpu9250_hdl,
                           (int16_t (*)[3]) &accel_raw, (float (*)[3]) &accel,
                           (int16_t (*)[3]) &gyro_raw, (float (*)[3]) &gyro,
                           (int16_t (*)[3]) &mag_raw, (float (*)[3]) &mag,
                           &len);

        if(ret != 0) {
            ESP_LOGE(MPU9250_TAG, "mpu read failed!");
            return ESP_FAIL;
        }

        int16_t raw_temp;
        float *temp_degrees = 0;
        ret = mpu9250_read_temperature(mpu9250_hdl, &raw_temp, temp_degrees);
        if(ret != 0) {
            ESP_LOGE(MPU9250_TAG, "temp read failed!");
            return ESP_FAIL;
        }

        ESP_LOGI(MPU9250_TAG,"%d/%d.\n", i + 1, MAX_ITERATIONS);
        ESP_LOGI(MPU9250_TAG,"acc x is %0.2fg.\n", accel[0]);
        ESP_LOGI(MPU9250_TAG,"acc y is %0.2fg.\n", accel[1]);
        ESP_LOGI(MPU9250_TAG,"acc z is %0.2fg.\n", accel[2]);
        ESP_LOGI(MPU9250_TAG,"gyro x is %0.2fdps.\n", gyro[0]);
        ESP_LOGI(MPU9250_TAG,"gyro y is %0.2fdps.\n", gyro[1]);
        ESP_LOGI(MPU9250_TAG,"gyro z is %0.2fdps.\n", gyro[2]);
        ESP_LOGI(MPU9250_TAG,"mag x is %0.2fuT.\n", mag[0]);
        ESP_LOGI(MPU9250_TAG,"mag y is %0.2fuT.\n", mag[1]);
        ESP_LOGI(MPU9250_TAG,"mag z is %0.2fuT.\n", mag[2]);
        ESP_LOGI(MPU9250_TAG,"temperature %0.2fC.\n", *temp_degrees);

        /* delay 1000 ms */
        mpu9250_interface_delay_ms(1000);
    }

    ret = mpu9250_mag_deinit(mpu9250_hdl);
    if(ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mag deinit failed!");
        return ESP_FAIL;
    }
    ret = mpu9250_deinit(mpu9250_hdl);
    if(ret != 0) {
        ESP_LOGE(MPU9250_TAG, "mpu deinit failed!");
        return ESP_FAIL;
    }



    return ESP_OK;
}