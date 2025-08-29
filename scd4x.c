#include "scd4x.h"
#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "SCD4x";

// -------- I2C addressing --------
static const uint8_t SCD4X_ADDR = 0x62;   // 7-bit I2C address

// -------- Timing & timeouts (datasheet-guided, conservative) --------
#define SCD4X_I2C_TIMEOUT_MS        1000   // I2C transaction timeout
#define SCD4X_DELAY_AFTER_STOP_MS    500   // wait after stop_periodic before other cmds
#define SCD4X_DELAY_AFTER_RESET_MS    10   // short settle after soft reset

// -------- CRC parameters (Sensirion format) --------
#define SCD4X_CRC_POLY  0x31   // x^8 + x^5 + x^4 + 1
#define SCD4X_CRC_INIT  0xFF

// -------- Command opcodes (16-bit, MSB first) --------
enum {
    SCD4X_CMD_START_PERIODIC    = 0x21B1,
    SCD4X_CMD_STOP_PERIODIC     = 0x3F86,
    SCD4X_CMD_READ_MEASUREMENT  = 0xEC05,
    SCD4X_CMD_GET_DATA_READY    = 0xE4B8,
    SCD4X_CMD_SOFT_RESET        = 0x3646,
};

// -------- Global default device instance (exported in the header) --------
SCD4x_t dev_scd4x = {
    .address       = 0x62,
    .i2c_port      = 0,
    .scl_speed_hz  = 400000,
    .i2c_bus_handle = NULL,
    .i2c_dev_handle = NULL,
    .initialized   = false,
    .periodic_on   = false,
};


//---------------------------------------------------------------------------------------------
/*=============================================================================================
==================================Internal helpers (file-local)===============================*/
//---------------------------------------------------------------------------------------------
// CRC-8 (Sensirion): polynomial 0x31, init 0xFF, MSB-first
static uint8_t scd4x_crc8_31(const uint8_t *p, size_t n)
{
    uint8_t crc = SCD4X_CRC_INIT;  // 0xFF

    if (p == NULL || n == 0) {
        return crc; // defined: CRC of empty buffer = init value
    }

    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ SCD4X_CRC_POLY); // 0x31
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


// Send a 16-bit command (MSB first), no payload/CRC.
// Used for simple commands like START/STOP/RESET.
// Returns ESP_ERR_INVALID_STATE if the global device is not initialized.
static esp_err_t scd4x_write_cmd(uint16_t cmd)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t frame[2];
    frame[0] = (uint8_t)(cmd >> 8);
    frame[1] = (uint8_t)(cmd & 0xFF);

    esp_err_t err = i2c_master_transmit(dev_scd4x.i2c_dev_handle,
                                        frame, sizeof(frame),
                                        SCD4X_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C TX cmd 0x%04X failed: %s", cmd, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/* Read n_words from a command that returns 16-bit words with CRC after each word.
Frame per word: [MSB][LSB][CRC]. Validates CRC (poly 0x31, init 0xFF).
Returns ESP_ERR_INVALID_STATE if not initialized, ESP_ERR_INVALID_ARG on bad params,
I2C errors from ESP-IDF, or ESP_ERR_INVALID_CRC on CRC failure.
*/
static esp_err_t scd4x_read_words(uint16_t cmd, uint16_t *out, size_t n_words)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out == NULL || n_words == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send command (MSB first)
    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    esp_err_t err = i2c_master_transmit(dev_scd4x.i2c_dev_handle,
                                        tx, sizeof(tx),
                                        SCD4X_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C TX cmd 0x%04X failed: %s", cmd, esp_err_to_name(err));
        return err;
    }

    // Receive words + CRC
    const size_t rx_len = n_words * 3; // 2 bytes data + 1 byte CRC per word
    uint8_t rx[rx_len];                // VLA (C99): safe for small n_words (typical 1 or 3)
    err = i2c_master_receive(dev_scd4x.i2c_dev_handle,
                             rx, rx_len,
                             SCD4X_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C RX for cmd 0x%04X failed: %s", cmd, esp_err_to_name(err));
        return err;
    }

    // CRC check and unpack
    for (size_t i = 0; i < n_words; ++i) {
        size_t k = i * 3;
        uint8_t msb = rx[k + 0];
        uint8_t lsb = rx[k + 1];
        uint8_t crc = rx[k + 2];

        uint8_t calc = scd4x_crc8_31(&rx[k], 2);
        if (calc != crc) {
            ESP_LOGE(TAG, "CRC mismatch (word %u, cmd 0x%04X): calc=0x%02X rx=0x%02X",
                     (unsigned)i, cmd, calc, crc);
            return ESP_ERR_INVALID_CRC;
        }
        out[i] = ((uint16_t)msb << 8) | (uint16_t)lsb;
    }

    return ESP_OK;
}


static inline void scd4x_convert_trh(uint16_t raw_t, uint16_t raw_rh,
                                     float *t_c, float *rh_pct)
{
    const float inv = 1.0f / 65535.0f;
    if (t_c)    *t_c    = -45.0f + 175.0f * (raw_t  * inv);
    if (rh_pct) {
        float rh = 100.0f * (raw_rh * inv);
        // Clamp por segurança numérica (às vezes sai 100.1% por arredondamento)
        if (rh < 0.0f)   rh = 0.0f;
        if (rh > 100.0f) rh = 100.0f;
        *rh_pct = rh;
    }
}
//---------------------------------------------------------------------------------------------
/*================================Internal helpers (file-local)================================
==============================================================================================*/
//---------------------------------------------------------------------------------------------




esp_err_t scd4x_init_default(uint8_t i2c_port,
                             gpio_num_t sda,
                             gpio_num_t scl,
                             uint32_t scl_speed_hz)
{
    // Reset local view of the global
    dev_scd4x.address        = SCD4X_ADDR;   // 0x62
    dev_scd4x.i2c_port       = i2c_port;
    dev_scd4x.scl_speed_hz   = scl_speed_hz;
    dev_scd4x.i2c_bus_handle = NULL;
    dev_scd4x.i2c_dev_handle = NULL;
    dev_scd4x.initialized    = false;
    dev_scd4x.periodic_on    = false;

    // Create I2C master bus
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = i2c_port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,   
        },
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    // Add SCD4x device on that bus
    i2c_device_config_t dev_cfg = {
        .device_address   = SCD4X_ADDR,           // 0x62 (7-bit)
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz     = scl_speed_hz,         // e.g., 400000
    };
    i2c_master_dev_handle_t dev = NULL;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        (void)i2c_del_master_bus(bus);
        return err;
    }

    //(Optional but nice) Probe the device quickly
    err = i2c_master_probe(bus, SCD4X_ADDR, SCD4X_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SCD4x not found at 0x%02X: %s", SCD4X_ADDR, esp_err_to_name(err));
        (void)i2c_master_bus_rm_device(dev);
        (void)i2c_del_master_bus(bus);
        return err;
    }
    
    // Publish into global and mark initialized
    dev_scd4x.i2c_bus_handle = bus;
    dev_scd4x.i2c_dev_handle = dev;
    dev_scd4x.initialized    = true;
    dev_scd4x.periodic_on    = false;

    ESP_LOGI(TAG, "SCD4x ready on I2C%d @ %u Hz, addr 0x%02X",
             i2c_port, (unsigned)scl_speed_hz, SCD4X_ADDR);
    return ESP_OK;
}



esp_err_t scd4x_start_periodic(void)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev_scd4x.periodic_on) {
        return ESP_ERR_INVALID_STATE; // already running
    }

    esp_err_t err = scd4x_write_cmd(SCD4X_CMD_START_PERIODIC);
    if (err == ESP_OK) {
        dev_scd4x.periodic_on = true;
        // First dataset will be ready after the sensor's internal interval (~5 s typical).
    }
    return err;
}


// -----------------------------------------------------------------------------
// Data-ready flag (periodic mode)
esp_err_t scd4x_get_data_ready(bool *ready)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL)
        return ESP_ERR_INVALID_STATE;
    if (ready == NULL)
        return ESP_ERR_INVALID_ARG;

    uint16_t status = 0;
    esp_err_t err = scd4x_read_words(SCD4X_CMD_GET_DATA_READY, &status, 1);
    if (err != ESP_OK) return err;

    *ready = (status != 0);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Leitura de CO2/T/RH convertida para unidade
esp_err_t scd4x_read(scd4x_sample_t *out)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL)
        return ESP_ERR_INVALID_STATE;
    if (out == NULL)
        return ESP_ERR_INVALID_ARG;

    // 3 words: CO2ppm, rawT, rawRH (cada com CRC).
    uint16_t w[3] = {0};
    esp_err_t err = scd4x_read_words(SCD4X_CMD_READ_MEASUREMENT, w, 3);
    if (err != ESP_OK) return err;

    out->co2_ppm = w[0];                       // ppm já em unidade final
    scd4x_convert_trh(w[1], w[2], &out->t_c, &out->rh_pct);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Stop periodic (retorna ao idle)
esp_err_t scd4x_stop_periodic(void)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL)
        return ESP_ERR_INVALID_STATE;
    if (!dev_scd4x.periodic_on)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = scd4x_write_cmd(SCD4X_CMD_STOP_PERIODIC);
    if (err == ESP_OK) {
        dev_scd4x.periodic_on = false;
        vTaskDelay(pdMS_TO_TICKS(SCD4X_DELAY_AFTER_STOP_MS));
    }
    return err;
}

// -----------------------------------------------------------------------------
// Soft reset 
esp_err_t scd4x_reset(void)
{
    if (!dev_scd4x.initialized || dev_scd4x.i2c_dev_handle == NULL)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = scd4x_write_cmd(SCD4X_CMD_SOFT_RESET);
    if (err == ESP_OK) {
        dev_scd4x.periodic_on = false;
        vTaskDelay(pdMS_TO_TICKS(SCD4X_DELAY_AFTER_RESET_MS));
    }
    return err;
}


