#ifndef _PERIPH_SSD1306_H_
#define _PERIPH_SSD1306_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"

#include "spidev.h"
#include "i2cdev.h"
#include "driver/gpio.h"
#include "fonts.h"

typedef enum {
	SSD1306_COLOR_BLACK = 0,
	SSD1306_COLOR_WHITE,
	SSD1306_COLOR_MAX
} ssd1306_color_t;

typedef enum {
	SSD1306_SIZE_128_32 = 0,
	SSD1306_SIZE_128_64,
	SSD1306_SIZE_MAX
} ssd1306_size_t;

typedef enum {
	SSD1306_COMM_MODE_I2C = 0,
	SSD1306_COMM_MODE_SPI,
	SSD1306_COMM_MODE_MAX
} ssd1306_comm_mode_t;

typedef struct {
	i2cdev_handle_t 	i2cdev;
	spidev_handle_t 	spidev;
	int 				dc;
	int 				rst;
	int 				cs;
} ssd1306_hw_info_t;

typedef struct {
	const char 				*tag;
	ssd1306_hw_info_t		hw_info;
	ssd1306_size_t 			size;
	ssd1306_comm_mode_t 	comm_mode;
	bool 					inverse;
} periph_ssd1306_cfg_t;

esp_periph_handle_t periph_ssd1306_init(periph_ssd1306_cfg_t *config);
esp_err_t periph_ssd1306_clear(esp_periph_handle_t periph);
esp_err_t periph_ssd1306_fill(esp_periph_handle_t periph, ssd1306_color_t color);
esp_err_t periph_ssd1306_write_char(esp_periph_handle_t periph, font_size_t font_size, uint8_t chr);
esp_err_t periph_ssd1306_write_string(esp_periph_handle_t periph, font_size_t font_size, uint8_t *str);
esp_err_t periph_ssd1306_draw_pixel(esp_periph_handle_t periph, uint8_t x, uint8_t y, ssd1306_color_t color);
esp_err_t periph_ssd1306_draw_line(esp_periph_handle_t periph, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color);
esp_err_t periph_ssd1306_draw_rectangle(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, ssd1306_color_t color);
esp_err_t periph_ssd1306_draw_circle(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t radius, ssd1306_color_t color);
esp_err_t periph_ssd1306_draw_image(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, uint8_t *img);
esp_err_t periph_ssd1306_set_position(esp_periph_handle_t periph, uint8_t x, uint8_t y);
esp_err_t periph_ssd1306_get_position(esp_periph_handle_t periph, uint8_t *x, uint8_t *y);

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_SSD1306_H_ */