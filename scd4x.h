#pragma once
/**
 * @file scd4x_min.h
 * @brief Minimal SCD4x (SCD40/SCD41) driver header for ESP-IDF using a global default device.
 *
 * Goal: smallest useful API to bring the sensor up, start periodic measurement, read, stop, reset.
 * No opcodes/CRC/timings are exposed here; they belong in the .c implementation.
 * All functions operate on the exported global instance `dev_scd4x`.
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

/** Converted measurement returned by scd4x_read(). */
typedef struct {
    uint16_t co2_ppm;  /**< CO2 in ppm */
    float    t_c;      /**< Temperature in °C */
    float    rh_pct;   /**< Relative humidity in % */
} scd4x_sample_t;

/** Lightweight device descriptor held globally (default device). */
typedef struct {
    uint8_t  address;        /**< I2C 7-bit address (SCD4x default: 0x62) */
    uint8_t  i2c_port;       /**< I2C port index (0/1) */
    uint32_t scl_speed_hz;   /**< I2C bus speed */
    i2c_master_bus_handle_t  i2c_bus_handle;  /**< ESP-IDF v5.2+ bus handle */
    i2c_master_dev_handle_t  i2c_dev_handle;  /**< ESP-IDF v5.2+ device handle */
    bool     initialized;    /**< True after successful init */
    bool     periodic_on;    /**< True if periodic measurement currently running */
} SCD4x_t;


extern SCD4x_t dev_scd4x;

/* ============================= */
/* ========== Minimal API ====== */
/* ============================= */

/**
 * @brief Initialize global default device and underlying I2C objects.
 *
 * After success, `dev_scd4x.initialized` will be true and `dev_scd4x.address` set to 0x62.
 *
 * @param i2c_port     I2C port index (0 or 1)
 * @param sda          SDA GPIO
 * @param scl          SCL GPIO
 * @param scl_speed_hz I2C clock (e.g., 100000 or 400000)
 * @return ESP_OK on success; ESP_ERR_* otherwise
 */
esp_err_t scd4x_init_default(uint8_t i2c_port,
                             gpio_num_t sda,
                             gpio_num_t scl,
                             uint32_t scl_speed_hz);

/**
 * @brief Start periodic measurement (sensor measures every ~5 s and caches the latest sample).
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized/already started; ESP_ERR_* on I2C errors
 */
esp_err_t scd4x_start_periodic(void);

/**
 * @brief Optional: Check if a new dataset is ready (periodic mode).
 * @param ready  [out] true if a fresh dataset is available, false otherwise
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized; ESP_ERR_* on I2C errors
 */
esp_err_t scd4x_get_data_ready(bool *ready);

/**
 * @brief Read the latest dataset (CO2/T/RH) converted to engineering units.
 * @param out  [out] destination sample
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized; ESP_ERR_* on I2C/CRC errors
 */
esp_err_t scd4x_read(scd4x_sample_t *out);

/**
 * @brief Stop periodic measurement (return sensor to idle).
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized/not running; ESP_ERR_* on I2C errors
 */
esp_err_t scd4x_stop_periodic(void);

/**
 * @brief Soft reset the sensor. Useful to recover from error states. Leaves sensor in idle.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not initialized; ESP_ERR_* on I2C errors
 */
esp_err_t scd4x_reset(void);

