/**
 * @file ina219.h
 * @brief INA219 FreeRTOS driver 
 *
 * @author     Artur Bereit
 * @date       2025-06-19
 * @version    1.0
 * @copyright  MIT License
 * @website    https://embedberight.com/
 */

#ifndef DRV_INA219_H
#define DRV_INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#if __has_include("drv_config.h")
#include "drv_config.h"
#endif

/**
 * @brief Maximum number of INA219 instances
 */
#ifndef INA219_MAX_INSTANCES
    #define INA219_MAX_INSTANCES 2
#endif

/**
 * @brief Default I2C address of INA219
 */
#define INA219_I2C_ADDR_DEFAULT 0x40

/**
 * @brief INA219 register addresses
 */
typedef enum ina219_register_e {
    INA219_REG_CONFIG     = 0x00,
    INA219_REG_SHUNT_V    = 0x01,
    INA219_REG_BUS_V      = 0x02,
    INA219_REG_POWER      = 0x03,
    INA219_REG_CURRENT    = 0x04,
    INA219_REG_CALIB      = 0x05
} ina219_register_t;

/**
 * @brief INA219 configuration structure
 */
typedef struct ina219_cfg_s {
    i2c_port_t i2c_num;       /*!< I2C port number */
    uint8_t i2c_addr;         /*!< INA219 I2C address (default 0x40) */
} ina219_cfg_t;

/**
 * @brief Create a static INA219 instance
 */
#define INA219_CONFIG_STATIC_INSTANCE(name, i2c, addr) \
    static ina219_cfg_t name = {                       \
        .i2c_num = i2c,                                   \
        .i2c_addr = addr                                  \
    }

/**
 * @brief Create a non-static INA219 instance
 */
#define INA219_CONFIG_INSTANCE(name, i2c, addr) \
    ina219_cfg_t name = {                       \
        .i2c_num = i2c,                            \
        .i2c_addr = addr                           \
    }

/**
 * @brief I2C configuration macro for INA219
 */
#define INA219_CONFIG_I2C(i2c_num, sda, scl)                        \
    i2c_config_t ina219_i2c_cfg = {                                    \
        .mode = I2C_MODE_MASTER,                                       \
        .sda_io_num = sda,                                             \
        .scl_io_num = scl,                                             \
        .sda_pullup_en = GPIO_PULLUP_ENABLE,                           \
        .scl_pullup_en = GPIO_PULLUP_ENABLE,                           \
        .master = {                                                    \ 
            .clk_speed = I2C_MASTER_FREQ_HZ,                           \
        },                                                             \         
        .clk_flags = 0                                                 \
    };                                                                 \
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &ina219_i2c_cfg));       \
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0))

/**
 * @brief Initialize INA219 driver
 *
 * @param instance_id Unique instance ID (0 to INA219_MAX_INSTANCES-1)
 * @param p_cfg Configuration structure pointer
 * @return esp_err_t
 */
esp_err_t ina219_init(uint8_t instance_id, const ina219_cfg_t* p_cfg);

/**
 * @brief Read a 16-bit register from INA219
 *
 * @param instance_id Instance ID
 * @param reg Register address
 * @param value Pointer to store read value
 * @param ticks_to_wait Timeout
 * @return esp_err_t
 */
esp_err_t ina219_read_register(uint8_t instance_id, ina219_register_t reg, uint16_t* value, TickType_t ticks_to_wait);

/**
 * @brief Write a 16-bit value to INA219 register
 *
 * @param instance_id Instance ID
 * @param reg Register address
 * @param value Value to write
 * @param ticks_to_wait Timeout
 * @return esp_err_t
 */
esp_err_t ina219_write_register(uint8_t instance_id, ina219_register_t reg, uint16_t value, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif

#endif // DRV_INA219_H
