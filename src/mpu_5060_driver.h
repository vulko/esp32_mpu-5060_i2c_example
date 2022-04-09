#include "stdio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define DRIVER_TAG "MPU-5060"
#define MPU_5060_ADDR 0x68
#define ACK_CHECK_EN 0x01
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define CONFIG_I2C_MASTER_PORT_NUM I2C_NUM_0
#define CONFIG_I2C_SLAVE_PORT_NUM I2C_NUM_1


void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

void checkErr(const char* msg, esp_err_t val) {
    if (val != ESP_OK) {
        ESP_LOGE(DRIVER_TAG, "%s: ESP Error! %d", msg, val);
    }
}

void delay_ms(uint32_t period_ms)
{
    vTaskDelay(period_ms / portTICK_PERIOD_MS);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    ESP_LOGD(DRIVER_TAG, "Starting write...");
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    checkErr("I2C: start", i2c_master_start(cmd));
    checkErr("I2C: write slave address", i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true));
    checkErr("I2C: write register address", i2c_master_write_byte(cmd, reg_addr, true));
    checkErr("I2C: write data", i2c_master_write(cmd, reg_data, length, true));
    checkErr("I2C: stop", i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 100 / portTICK_RATE_MS);
    checkErr("I2C: begin", ret);
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? 0 : 1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    ESP_LOGD(DRIVER_TAG, "Starting read...");
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    checkErr("I2C: start",
             i2c_master_start(cmd));
    checkErr("I2C: write address",
             i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true));
    checkErr("I2C: write register address",
             i2c_master_write_byte(cmd, reg_addr, true));
    // repeated start to enable read mode
    checkErr("I2C: re-start",
             i2c_master_start(cmd));
    checkErr("I2C: write address in read mode",
             i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true));
    if (length > 1) {
        checkErr("I2C: read data bytes",
                 i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK));
    }
    checkErr("I2C: read last data byte",
             i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_LAST_NACK));
    checkErr("I2C: stop",
             i2c_master_stop(cmd));

    ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 100 / portTICK_RATE_MS);
    checkErr("I2C: begin", ret);
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? 0 : 1;
}

void initialize() {
    uint16_t X, Y, Z;
    uint16_t WX, WY, WZ;
    uint8_t val;
    volatile uint32_t i32;

    delay_ms(100);

    //Датчик тактируется от встроенного 8Мгц осциллятора
    i2c_reg_write(MPU_5060_ADDR, 0x6B, (uint8_t) 0x00, 1);  // Register_PWR_M1 = 0, Disable sleep mode
    //Выполнить очистку встроенных регистров датчика
    i2c_reg_write(MPU_5060_ADDR, 0x6A, (uint8_t) 0x01, 1);  // Register_UsCtrl = 1

    delay_ms(100);
    
    while (true) {
        i2c_reg_read(MPU_5060_ADDR, 0x3B, &val, 1);
        X = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x3C, &val, 1);
        X |= val;

        i2c_reg_read(MPU_5060_ADDR, 0x3D, &val, 1);
        Y = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x3E, &val, 1);
        Y |= val;

        i2c_reg_read(MPU_5060_ADDR, 0x3F, &val, 1);
        Z = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x40, &val, 1);
        Z |= val;

        i2c_reg_read(MPU_5060_ADDR, 0x43, &val, 1);
        WX = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x44, &val, 1);
        WX |= val;

        i2c_reg_read(MPU_5060_ADDR, 0x45, &val, 1);
        WY = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x46, &val, 1);
        WY |= val;

        i2c_reg_read(MPU_5060_ADDR, 0x47, &val, 1);
        WZ = val << 8;
        i2c_reg_read(MPU_5060_ADDR, 0x48, &val, 1);
        WZ |= val;

        ESP_LOGE(DRIVER_TAG, "X = %d, Y = %d, Z = %d", X, Y, Z);
        ESP_LOGE(DRIVER_TAG, "WX = %d, WY = %d, WZ = %d", WX, WY, WZ);
        delay_ms(1000);
    }

}