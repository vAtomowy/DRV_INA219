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

#if __has_include ("drv_config.h")
#include "drv_config.h"
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Maximum number of INA219 instances
 */
#ifndef INA219_MAX_INSTANCES
    #define INA219_MAX_INSTANCES 2
#endif

/**
 * @brief Default I2C transmission speed of INA219
 */
#define INA219_DEFAULT_SPEED_HZ 100000

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
 * @brief INA219 bus voltage range settings
 */
typedef enum ina219_bvoltagerange_e {
    INA219_BVOLTAGERANGE_16V = 0x0000, // 0–16V
    INA219_BVOLTAGERANGE_32V = 0x2000  // 0–32V
} ina219_bvoltagerange_t;

#define INA219_BVOLTAGERANGE_MASK 0x2000

/**
 * @brief INA219 programmable gain amplifier gain settings
 */
typedef enum ina219_gain_e {
    INA219_GAIN_1_40MV  = 0x0000,
    INA219_GAIN_2_80MV  = 0x0800,
    INA219_GAIN_4_160MV = 0x1000,
    INA219_GAIN_8_320MV = 0x1800
} ina219_gain_t;

#define INA219_GAIN_MASK 0x1800

/**
 * @brief INA219 bus ADC resolution/averaging settings
 */
typedef enum ina219_adc_busres_e {
    INA219_BADCRES_9BIT         = 0x0000,
    INA219_BADCRES_10BIT        = 0x0080,
    INA219_BADCRES_11BIT        = 0x0100,
    INA219_BADCRES_12BIT        = 0x0180,
    INA219_BADCRES_12BIT_2S     = 0x0480,
    INA219_BADCRES_12BIT_4S     = 0x0500,
    INA219_BADCRES_12BIT_8S     = 0x0580,
    INA219_BADCRES_12BIT_16S    = 0x0600,
    INA219_BADCRES_12BIT_32S    = 0x0680,
    INA219_BADCRES_12BIT_64S    = 0x0700,
    INA219_BADCRES_12BIT_128S   = 0x0780
} ina219_badcres_t;

#define INA219_BADCRES_MASK 0x0780

/**
 * @brief INA219 shunt ADC resolution/averaging settings
 */
typedef enum ina219_adc_shuntres_e {
    INA219_SADCRES_9BIT_1S      = 0x0000,
    INA219_SADCRES_10BIT_1S     = 0x0008,
    INA219_SADCRES_11BIT_1S     = 0x0010,
    INA219_SADCRES_12BIT_1S     = 0x0018,
    INA219_SADCRES_12BIT_2S     = 0x0048,
    INA219_SADCRES_12BIT_4S     = 0x0050,
    INA219_SADCRES_12BIT_8S     = 0x0058,
    INA219_SADCRES_12BIT_16S    = 0x0060,
    INA219_SADCRES_12BIT_32S    = 0x0068,
    INA219_SADCRES_12BIT_64S    = 0x0070,
    INA219_SADCRES_12BIT_128S   = 0x0078
} ina219_sadcres_t;

#define INA219_SADCRES_MASK 0x0078

/**
 * @brief INA219 operating mode settings
 */
typedef enum ina219_mode_e {
    INA219_MODE_POWERDOWN             = 0x00,
    INA219_MODE_SVOLT_TRIGGERED       = 0x01,
    INA219_MODE_BVOLT_TRIGGERED       = 0x02,
    INA219_MODE_SANDBVOLT_TRIGGERED   = 0x03,
    INA219_MODE_ADCOFF                = 0x04,
    INA219_MODE_SVOLT_CONTINUOUS      = 0x05,
    INA219_MODE_BVOLT_CONTINUOUS      = 0x06,
    INA219_MODE_SANDBVOLT_CONTINUOUS  = 0x07
} ina219_mode_t;

#define INA219_MODE_MASK 0x0007

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
#define INA219_CONFIG_STATIC_INSTANCE(name, i2c, addr)    \
    static ina219_cfg_t name = {                          \
        .i2c_num = i2c,                                   \
        .i2c_addr = addr                                  \
    }

/**
 * @brief Create a non-static INA219 instance
 */
#define INA219_CONFIG_INSTANCE(name, i2c, addr)    \
    ina219_cfg_t name = {                          \
        .i2c_num = i2c,                            \
        .i2c_addr = addr                           \
    }

/**
 * @brief I2C configuration macro for INA219
 */
#define INA219_CONFIG_I2C(i2c_num, sda, scl)                           \
    i2c_config_t ina219_i2c_cfg = {                                    \
        .mode = I2C_MODE_MASTER,                                       \
        .sda_io_num = sda,                                             \
        .scl_io_num = scl,                                             \
        .sda_pullup_en = GPIO_PULLUP_ENABLE,                           \
        .scl_pullup_en = GPIO_PULLUP_ENABLE,                           \
        .master = {                                                    \ 
            .clk_speed = INA219_DEFAULT_SPEED_HZ,                      \
        },                                                             \         
        .clk_flags = 0                                                 \
    };                                                                 \
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &ina219_i2c_cfg));       \
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0))

/**
 * @brief Initialize an INA219 instance
 *
 * This function initializes the INA219 device with the specified configuration.
 *
 * @param instance_id Unique identifier for the INA219 instance (range: 0 to INA219_MAX_INSTANCES - 1)
 * @param p_cfg Pointer to the configuration structure containing I2C parameters
 * @return esp_err_t Returns ESP_OK on success or an error code on failure
 */
esp_err_t ina219_init(uint8_t instance_id, const ina219_cfg_t* p_cfg);

/**
 * @brief Read a 16-bit register value from the INA219 device
 *
 * This function reads a 16-bit value from the specified INA219 register.
 *
 * @param instance_id Identifier of the INA219 instance to read from
 * @param reg Register address to read (use ina219_register_t enum)
 * @param value Pointer to a variable where the read value will be stored
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the read was successful or an error code otherwise
 */
//esp_err_t ina219_read_register(uint8_t instance_id, ina219_register_t reg, uint16_t* value, TickType_t ticks_to_wait);

/**
 * @brief Write a 16-bit value to a register in the INA219 device
 *
 * This function writes a 16-bit value to the specified INA219 register.
 *
 * @param instance_id Identifier of the INA219 instance to write to
 * @param reg Register address to write (use ina219_register_t enum)
 * @param value 16-bit value to write to the register
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the write was successful or an error code otherwise
 */
//esp_err_t ina219_write_register(uint8_t instance_id, ina219_register_t reg, uint16_t value, TickType_t ticks_to_wait);

/**
 * @brief Configure the INA219 device by writing to its configuration register
 *
 * This function writes a configuration value to the INA219 configuration register.
 *
 * @param instance_id Identifier of the INA219 instance to configure
 * @param config 16-bit configuration value to set
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the configuration was successful or an error code otherwise
 */
esp_err_t ina219_configure(uint8_t instance_id, uint16_t config, TickType_t ticks_to_wait);

/**
 * @brief Write calibration data to the INA219 calibration register
 *
 * This function sets the INA219's internal calibration register. Calibration affects
 * how the device scales shunt voltage readings to current and power values. The calibration
 * value is typically calculated based on the shunt resistor value and expected current range.
 *
 * @param instance_id Identifier of the INA219 instance to calibrate
 * @param config 16-bit calibration value to write into the INA219_CAL register
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the calibration value was successfully written, or an error code otherwise
 */
esp_err_t ina219_calibration(uint8_t instance_id, uint16_t config, TickType_t ticks_to_wait);

/**
 * @brief Read the bus voltage from the INA219 device
 *
 * This function reads the bus voltage register, which represents the voltage on the bus side of the shunt resistor.
 *
 * @param instance_id Identifier of the INA219 instance to read from
 * @param value Pointer to a variable where the bus voltage value will be stored
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the read was successful or an error code otherwise
 */
esp_err_t ina219_read_bus_voltage(uint8_t instance_id, uint16_t* value, TickType_t ticks_to_wait);

/**
 * @brief Read the shunt voltage from the INA219 device
 *
 * This function reads the shunt voltage register, which represents the voltage drop across the shunt resistor.
 *
 * @param instance_id Identifier of the INA219 instance to read from
 * @param value Pointer to a variable where the shunt voltage value will be stored
 * @param ticks_to_wait Maximum time to wait for the semaphore (in FreeRTOS ticks)
 * @return esp_err_t Returns ESP_OK if the read was successful or an error code otherwise
 */
esp_err_t ina219_read_shunt_voltage(uint8_t instance_id, uint16_t* value, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif

#endif // DRV_INA219_H
