/**
 * @file ina219.c
 * @brief INA219 FreeRTOS driver 
 *
 * @author     Artur Bereit
 * @date       2025-06-19
 * @version    1.0
 * @copyright  MIT License
 * @website    https://embedberight.com/
 */

#include "ina219.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *DRV_INA219_TAG = "DRV_INA219";

#define INA219_CFG_ERROR_STR         "ina219 configuration pointer error"
#define INA219_INSTANCE_ID_ERROR_STR "ina219 instance id error"
#define INA219_I2C_NUM_ERROR_STR     "ina219 i2c number error"
#define INA219_SEM_ERR_STR           "ina219 semaphore error"

typedef struct ina219_obj_s {
    ina219_cfg_t cfg;
    SemaphoreHandle_t mux;
    uint8_t instance_id;
} ina219_obj_t;

static ina219_obj_t* p_ina219_obj[INA219_MAX_INSTANCES] = {0};

static esp_err_t ina219_write_register(ina219_obj_t* p_obj, uint8_t reg, uint16_t value, TickType_t ticks_to_wait) {
    uint8_t data[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_write_to_device(p_obj->cfg.i2c_num, p_obj->cfg.i2c_addr, data, 3, ticks_to_wait);
}

static esp_err_t ina219_read_register(ina219_obj_t* p_obj, uint8_t reg, uint16_t* value, TickType_t ticks_to_wait) {
    uint8_t data[2];
    esp_err_t err = i2c_master_write_read_device(p_obj->cfg.i2c_num, p_obj->cfg.i2c_addr, &reg, 1, data, 2, ticks_to_wait);
    if (err == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return err;
}

esp_err_t ina219_init(uint8_t instance_id, const ina219_cfg_t* p_cfg) {
    esp_err_t err = ESP_OK;
    ina219_obj_t* p_obj = NULL;

    ESP_RETURN_ON_FALSE(p_cfg != NULL, ESP_ERR_INVALID_ARG, DRV_INA219_TAG, INA219_CFG_ERROR_STR);
    ESP_RETURN_ON_FALSE(instance_id < INA219_MAX_INSTANCES, ESP_ERR_INVALID_ARG, DRV_INA219_TAG, INA219_INSTANCE_ID_ERROR_STR);
    ESP_RETURN_ON_FALSE(p_cfg->i2c_num >= 0 && p_cfg->i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, DRV_INA219_TAG, INA219_I2C_NUM_ERROR_STR);

    if(p_ina219_obj[instance_id] != NULL) return ESP_FAIL;

    p_obj = (ina219_obj_t*)calloc(1, sizeof(ina219_obj_t));
    if(!p_obj) return ESP_ERR_NO_MEM;

    *p_obj = (ina219_obj_t){
        .cfg = *p_cfg,
        .instance_id = instance_id,
    };

    p_obj->mux = xSemaphoreCreateMutex();
    if (!p_obj->mux) {
        ESP_LOGE(DRV_INA219_TAG, INA219_SEM_ERR_STR);
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    p_ina219_obj[instance_id] = p_obj;
    return ESP_OK;

cleanup:
    if (p_obj->mux) vSemaphoreDelete(p_obj->mux);
    free(p_obj);
    return err;
}

esp_err_t ina219_read_bus_voltage(uint8_t instance_id, uint16_t* value, TickType_t ticks_to_wait) {
    if (p_ina219_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(p_ina219_obj[instance_id]->mux, ticks_to_wait) == pdFALSE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = ina219_read_register(p_ina219_obj[instance_id], INA219_REG_BUS_V, value, ticks_to_wait);
    xSemaphoreGive(p_ina219_obj[instance_id]->mux);
    return err;
}

esp_err_t ina219_read_shunt_voltage(uint8_t instance_id, uint16_t* value, TickType_t ticks_to_wait) {
    if (p_ina219_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(p_ina219_obj[instance_id]->mux, ticks_to_wait) == pdFALSE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = ina219_read_register(p_ina219_obj[instance_id], INA219_REG_SHUNT_V, value, ticks_to_wait);
    xSemaphoreGive(p_ina219_obj[instance_id]->mux);
    return err;
}

esp_err_t ina219_configure(uint8_t instance_id, uint16_t config, TickType_t ticks_to_wait) {
    if (p_ina219_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(p_ina219_obj[instance_id]->mux, ticks_to_wait) == pdFALSE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = ina219_write_register(p_ina219_obj[instance_id], INA219_REG_CONFIG, config, ticks_to_wait);
    xSemaphoreGive(p_ina219_obj[instance_id]->mux);
    return err;
}

esp_err_t ina219_calibration(uint8_t instance_id, uint16_t config, TickType_t ticks_to_wait) {
    if (p_ina219_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(p_ina219_obj[instance_id]->mux, ticks_to_wait) == pdFALSE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = ina219_write_register(p_ina219_obj[instance_id], INA219_REG_CALIB, config, ticks_to_wait);
    xSemaphoreGive(p_ina219_obj[instance_id]->mux);
    return err;
}
