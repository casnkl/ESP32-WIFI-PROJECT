#ifndef LCD_BMP_H
#define LCD_BMP_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// ================= I2C =================
void i2c_master_init(void);
void i2c_data_init(void);

// ================= LCD =================
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);

// ================= BME280 =================
void bmp280_init(void);
esp_err_t bmp280_read_raw(void);
esp_err_t cal_t_var_read(void);
esp_err_t cal_p_var_read(void);
double real_temp(void);
double get_real_pressure(void);

#endif // LCD_BMp_H
