#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "driver/i2c_master.h"
#include "LCD_bmp.h"

// ===================== I2C and device settings =====================
#define I2C_MASTER_SCL_IO 22          // I2C clock pin
#define I2C_MASTER_SDA_IO 21          // I2C data pin
#define I2C_MASTER_FREQ_HZ 100000     // I2C frequency

#define LCD_ADDR 0x27                 // I2C address of the LCD
#define BME280_ADDR 0x76              // I2C address of BME280 sensor

#define crtl_mean_addr 0xF4           // BME280 control register
#define crtl_mean 0x57                // Oversampling 2x temperature, 16x pressure, normal mode
#define config_addr 0xF5              // BME280 configuration register
#define config 0x10                   // Standby and filter configuration

#define BME280_REG_DATA_ALL 0xF7      // Starting register for raw sensor data
#define dig_T_start 0x88              // Temperature calibration data start
#define dig_P_start 0x8E              // Pressure calibration data start

// ===================== LCD data structure =====================
typedef union {
    uint8_t data;
    struct {
        uint8_t LCD_RS_BIT : 1;       // Register select bit (0=command, 1=data)
        uint8_t LCD_R_W_b : 1;        // Read/Write bit (0=write, 1=read)
        uint8_t LCD_ENABLE_BIT_b : 1; // Enable pulse for LCD
        uint8_t LCD_BACKLIGHT_b : 1;  // Backlight on/off
        uint8_t data_b0 : 1;          // Data bit 0 (4-bit mode)
        uint8_t data_b1 : 1;          // Data bit 1
        uint8_t data_b2 : 1;          // Data bit 2
        uint8_t data_b3 : 1;          // Data bit 3
    } bits;
} data_send;

static data_send d; // Global LCD data variable

// ===================== BME280 raw data and calibration =====================
typedef struct {
    int32_t raw_press;
    int32_t raw_temp;
} bme280_raw_data_t;

bme280_raw_data_t raw; // Global raw data

typedef struct {
    uint16_t T1;
    int16_t T2;
    int16_t T3;
} temp_cal_var;

RTC_DATA_ATTR temp_cal_var cal_t; // Temperature calibration, stored in RTC memory

typedef struct {
    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;
} press_cal_var;

RTC_DATA_ATTR press_cal_var cal_p; // Pressure calibration, stored in RTC memory

static volatile int32_t t_fine = 0; // Intermediate value used in temperature & pressure compensation

// ===================== I2C bus and device handles =====================
i2c_master_bus_handle_t bus_master;
i2c_master_dev_handle_t lcd_dev;
i2c_master_dev_handle_t bme_dev;

// ===================== I2C initialization =====================
void i2c_master_init() {
    // Configure the I2C master bus
    i2c_master_bus_config_t master_conf = {
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .i2c_port = -1,                    // Let the system choose an available port
        .flags.enable_internal_pullup = 1, // Enable internal pull-ups
        .glitch_ignore_cnt = 7,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_conf, &bus_master));

    // Add LCD device to the I2C bus
    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .flags.disable_ack_check = false,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_master, &dev_conf, &lcd_dev));

    // Add BME280 device to the I2C bus
    i2c_device_config_t dev_conf2 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .flags.disable_ack_check = false,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_master, &dev_conf2, &bme_dev));
}

// ===================== LCD initialization =====================
void i2c_data_init() {
    d.data = 0;
    d.bits.LCD_BACKLIGHT_b = 1; // Turn on backlight
    d.bits.LCD_R_W_b = 0;       // Set write mode
}

// Low-level write to LCD via I2C
static esp_err_t lcd_write_byte(uint8_t data) {
    return i2c_master_transmit(lcd_dev, &data, 1, pdMS_TO_TICKS(1000));
}

// Enable pulse for 4-bit communication
static void lcd_strobe(void) {
    d.bits.LCD_ENABLE_BIT_b = 1;
    lcd_write_byte(d.data); 
    vTaskDelay(pdMS_TO_TICKS(1));
    d.bits.LCD_ENABLE_BIT_b = 0;
    lcd_write_byte(d.data); 
    vTaskDelay(pdMS_TO_TICKS(1));
}

// Send command to LCD
void lcd_send_cmd(uint8_t cmd) {
    d.bits.LCD_RS_BIT = 0; // Command mode

    // Send upper nibble (bits 7-4)
    d.bits.data_b0 = (cmd >> 4) & 1;
    d.bits.data_b1 = (cmd >> 5) & 1;
    d.bits.data_b2 = (cmd >> 6) & 1;
    d.bits.data_b3 = (cmd >> 7) & 1;
    lcd_strobe();

    // Send lower nibble (bits 3-0)
    d.bits.data_b0 = (cmd >> 0) & 1;
    d.bits.data_b1 = (cmd >> 1) & 1;
    d.bits.data_b2 = (cmd >> 2) & 1;
    d.bits.data_b3 = (cmd >> 3) & 1;
    lcd_strobe();
}

// Send data (character) to LCD
void lcd_send_data(uint8_t data) {
    d.bits.LCD_RS_BIT = 1; // Data mode

    // Upper nibble
    d.bits.data_b0 = (data >> 4) & 1;
    d.bits.data_b1 = (data >> 5) & 1;
    d.bits.data_b2 = (data >> 6) & 1;
    d.bits.data_b3 = (data >> 7) & 1;
    lcd_strobe();

    // Lower nibble
    d.bits.data_b0 = (data >> 0) & 1;
    d.bits.data_b1 = (data >> 1) & 1;
    d.bits.data_b2 = (data >> 2) & 1;
    d.bits.data_b3 = (data >> 3) & 1;
    lcd_strobe();
}

// Send upper nibble only (used during initialization)
static void lcd_pulse_upper_nibble(uint8_t data_val) {
    d.bits.LCD_RS_BIT = 0; 
    d.bits.data_b0 = (data_val >> 4) & 1;
    d.bits.data_b1 = (data_val >> 5) & 1;
    d.bits.data_b2 = (data_val >> 6) & 1;
    d.bits.data_b3 = (data_val >> 7) & 1;
    lcd_strobe();
}

// ===================== LCD high-level functions =====================
void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for LCD to power up

    // Initialization sequence for 4-bit mode
    lcd_pulse_upper_nibble(0x30); 
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_pulse_upper_nibble(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_pulse_upper_nibble(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Switch to 4-bit mode
    lcd_pulse_upper_nibble(0x20); 
    vTaskDelay(pdMS_TO_TICKS(1));

    lcd_send_cmd(0x28); // 2 lines, 5x8 dots
    lcd_send_cmd(0x0C); // Display ON, cursor OFF
    lcd_send_cmd(0x06); // Cursor move direction
    lcd_send_cmd(0x01); // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_clear() {
    lcd_send_cmd(0x01); // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_cmd(0x80 | (col + row_offsets[row])); // Move cursor to col,row
}

void lcd_print(const char* str) {
    while(*str) lcd_send_data(*str++);
}

// ===================== BME280 low-level I2C =====================
static esp_err_t bme280_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
}

// Read multiple bytes from BME280
static esp_err_t bme280_read_data(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
}

// ===================== BME280 high-level =====================
// Initialize BME280
void bmp280_init() {
    bme280_write_byte(bme_dev, 0xE0, 0xB6); // Soft reset
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(bme280_write_byte(bme_dev, config_addr, config));
    ESP_ERROR_CHECK(bme280_write_byte(bme_dev, crtl_mean_addr, crtl_mean));
}

// Read raw pressure and temperature from BME280
esp_err_t bmp280_read_raw() {
    uint8_t data[8]; 
    esp_err_t err = bme280_read_data(bme_dev, BME280_REG_DATA_ALL, data, 8);
    if (err != ESP_OK) return err;

    // Reconstruct 20-bit pressure
    raw.raw_press = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    // Reconstruct 20-bit temperature
    raw.raw_temp = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

    return ESP_OK;
}

// ===================== Helper functions =====================
static inline int16_t concat_s16(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

static inline uint16_t concat_u16(uint8_t msb, uint8_t lsb) {
    return (uint16_t)((msb << 8) | lsb);
}

// Read temperature calibration data
esp_err_t cal_t_var_read() {
    uint8_t data[6];
    esp_err_t err = bme280_read_data(bme_dev, dig_T_start, data, 6);
    if (err != ESP_OK) return err;

    cal_t.T1 = concat_u16(data[1], data[0]);
    cal_t.T2 = concat_s16(data[3], data[2]);
    cal_t.T3 = concat_s16(data[5], data[4]);

    return ESP_OK;
}

// Convert raw temperature to real temperature in °C
double real_temp() {
    int32_t temp, var1, var2;

    var1 = ((((raw.raw_temp >> 3) - (cal_t.T1 << 1))) * cal_t.T2) >> 11;
    var2 = (((((raw.raw_temp >> 4) - cal_t.T1) * ((raw.raw_temp >> 4) - cal_t.T1)) >> 12) * cal_t.T3) >> 14;
    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;

    return (double)temp / 100;
}

// Read pressure calibration data
esp_err_t cal_p_var_read() {
    uint8_t data[18];
    esp_err_t err = bme280_read_data(bme_dev, dig_P_start, data, 18);
    if (err != ESP_OK) return err;

    cal_p.P1 = concat_u16(data[1], data[0]);
    cal_p.P2 = concat_s16(data[3], data[2]);
    cal_p.P3 = concat_s16(data[5], data[4]);
    cal_p.P4 = concat_s16(data[7], data[6]);
    cal_p.P5 = concat_s16(data[9], data[8]);
    cal_p.P6 = concat_s16(data[11], data[10]);
    cal_p.P7 = concat_s16(data[13], data[12]);
    cal_p.P8 = concat_s16(data[15], data[14]);
    cal_p.P9 = concat_s16(data[17], data[16]);

    return ESP_OK;
}

// Calculate real pressure in hPa using compensation formula
double get_real_pressure() {
    int64_t var1, var2, p;

    // t_fine must be calculated from temperature first
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal_p.P6;
    var2 += ((var1 * (int64_t)cal_p.P5) << 17);
    var2 += ((int64_t)cal_p.P4) << 35;
    var1 = ((var1 * var1 * (int64_t)cal_p.P3) >> 8) + ((var1 * (int64_t)cal_p.P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)cal_p.P1) >> 33;

    if (var1 == 0) return 0; // Avoid division by zero

    p = 1048576 - raw.raw_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal_p.P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal_p.P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal_p.P7) << 4);

    return (double)p / 256.0 / 100.0; // Convert to hPa
}
