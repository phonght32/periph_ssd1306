#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "periph_ssd1306.h"

#define TIMEOUT_MS 							100

#define SSD1306_ADDR						(0x3C)
#define SSD1306_REG_DATA_ADDR				0x40
#define SSD1306_REG_CMD_ADDR				0x00
/*!<  */
#define SSD1306_SET_CONTRAST				0x81 		/*!< 0x81 + 0x00~0xFF Contrast ... reset = 0x7F */
#define SSD1306_DISPLAYALLON_RESUME 		0xA4		/*!< Resume to RAM content display */
#define SSD1306_DISPLAYALLON_IGNORE 		0xA5		/*!< Ignore RAM content display */
#define SSD1306_DISPLAY_NORMAL 				0xA6		/*!< White: 1; Black: 0 */
#define SSD1306_DISPLAY_INVERSE 			0xA7		/*!< White: 0; Black: 1 */
#define SSD1306_DISPLAY_OFF 				0xAE		/*!< Screen OFF */
#define SSD1306_DISPLAY_ON 					0xAF		/*!< Screen ON */

#define SSD1306_SET_MEMORYMODE 				0x20 		/*!< 0x20 + 0x00: horizontal; 0x01: vertical; 0x02: page */
#define SSD1306_SET_MEMORYMODE_HOR 			0x00
#define SSD1306_SET_MEMORYMODE_VER 			0x01
#define SSD1306_SET_MEMORYMODE_PAGE 		0x02

#define SSD1306_SET_COLUMN_ADDR 			0x21  		/*!< 0x21 + 0~127 + 0~127: colum start address + column end address */
#define SSD1306_SET_PAGE_ADDR 				0x22		/*!< 0x22 + 0~7 + 0~7: page start address + page end address */

#define SSD1306_SET_STARTLINE_ZERO 			0x40
#define SSD1306_SET_SEGREMAP_NORMAL  		0xA0
#define SSD1306_SET_SEGREMAP_INV 			0xA1
#define SSD1306_SET_MULTIPLEX 				0XA8
#define SSD1306_COMSCAN_INC 				0xC0
#define SSD1306_COMSCAN_DEC 				0xC8
#define SSD1306_SET_DISPLAYOFFSET 			0xD3
#define SSD1306_SET_COMPINS 				0xDA

#define SSD1306_SET_CLKDIV 					0xD5
#define SSD1306_SET_PRECHARGE 				0xD9
#define SSD1306_SET_COMDESELECT 			0xDB
#define SSD1306_NOP 						0xE3

#define SSD1306_CHARGEPUMP 					0x8D
#define SSD1306_CHARGEPUMP_ON 				0x14
#define SSD1306_CHARGEPUMP_OFF 				0x10

#define mutex_lock(x)			while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 		xSemaphoreGive(x)
#define mutex_create()			xSemaphoreCreateMutex()
#define mutex_destroy(x) 		vQueueDelete(x)

#define VALIDATE_SSD1306(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_SSD1306)) {		\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_SSD1306");													\
	return ret;																					\
}

#define SSD1306_CHECK(a, str, action) if(!(a)) {							\
	ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}

typedef esp_err_t (*init_func)(ssd1306_hw_info_t hw_info);
typedef esp_err_t (*write_cmd_func)(ssd1306_hw_info_t hw_info, uint8_t cmd);
typedef esp_err_t (*write_data_func)(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len);

typedef struct periph_ssd1306 {
	ssd1306_hw_info_t		hw_info;
	ssd1306_size_t 			size;
	ssd1306_comm_mode_t 	comm_mode;
	uint8_t					height;
	uint8_t					width;
	write_data_func			_write_data;
	write_cmd_func 			_write_cmd;
	uint8_t  				*buf[2];
	uint32_t 				buf_len;
	uint8_t					buf_idx;
	uint8_t 				cur_x;
	uint8_t					cur_y;
	bool 					inverse;
	bool					is_started;
	SemaphoreHandle_t		lock;
} periph_ssd1306_t;

static const char *TAG = "SSD1306";
static esp_periph_handle_t g_ssd1306;

static esp_err_t _init_i2c(ssd1306_hw_info_t hw_info)
{
	return ESP_OK;
}

static esp_err_t _init_spi(ssd1306_hw_info_t hw_info)
{
	return ESP_OK;
}

static esp_err_t _i2c_write_data(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len)
{
	i2cdev_set_addr(hw_info.i2cdev, SSD1306_ADDR);
	SSD1306_CHECK(!i2cdev_write(hw_info.i2cdev, SSD1306_REG_DATA_ADDR, data, len), "write data error", return ESP_FAIL);

	return ESP_OK;
}

static esp_err_t _spi_write_data(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len)
{
	return ESP_OK;
}

static esp_err_t _i2c_write_cmd(ssd1306_hw_info_t hw_info, uint8_t cmd)
{
	i2cdev_set_addr(hw_info.i2cdev, SSD1306_ADDR);
	SSD1306_CHECK(!i2cdev_write(hw_info.i2cdev, SSD1306_REG_CMD_ADDR, &cmd, 1), "write cmd error", return ESP_FAIL);

	return ESP_OK;
}

static esp_err_t _spi_write_cmd(ssd1306_hw_info_t hw_info, uint8_t cmd)
{
	return ESP_OK;
}

static init_func _get_init_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _init_i2c;
	} else {
		return _init_spi;
	}

	return NULL;
}

static write_data_func _get_write_data_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _i2c_write_data;
	} else {
		return _spi_write_data;
	}

	return NULL;
}

static write_cmd_func _get_write_cmd_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _i2c_write_cmd;
	} else {
		return _spi_write_cmd;
	}

	return NULL;
}

static uint8_t _get_screen_width(ssd1306_size_t size)
{
	if (size == SSD1306_SIZE_128_32) {
		return 128;
	} else {
		return 128;
	}
}

static uint8_t _get_screen_height(ssd1306_size_t size)
{
	if (size == SSD1306_SIZE_128_32) {
		return 32;
	} else {
		return 64;
	}
}

static void _ssd1306_cleanup(esp_periph_handle_t periph)
{
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	free(periph_ssd1306->buf[0]);
	free(periph_ssd1306->buf[1]);
	free(periph_ssd1306);
	free(periph);
}

static void _draw_pixel(periph_ssd1306_t *periph_ssd1306, uint8_t x, uint8_t y, ssd1306_color_t color)
{
	if (periph_ssd1306->inverse) {
		if (color == SSD1306_COLOR_WHITE) {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] &= ~ (1 << (y % 8));
		} else {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] |= (1 << (y % 8));
		}
	} else {
		if (color == SSD1306_COLOR_WHITE) {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] |= (1 << (y % 8));
		} else {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] &= ~ (1 << (y % 8));
		}
	}
}

static void _draw_line(periph_ssd1306_t *periph_ssd1306, uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, ssd1306_color_t color)
{
	int32_t deltaX = abs(x_end - x_start);
	int32_t deltaY = abs(y_end - y_start);
	int32_t signX = ((x_start < x_end) ? 1 : -1);
	int32_t signY = ((y_start < y_end) ? 1 : -1);
	int32_t error = deltaX - deltaY;
	int32_t error2;

	if (periph_ssd1306->inverse) {
		if (color == SSD1306_COLOR_WHITE) {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x_end + (y_end / 8)*periph_ssd1306->width] &= ~ (1 << (y_end % 8));
		} else {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x_end + (y_end / 8)*periph_ssd1306->width] |= (1 << (y_end % 8));
		}
	} else {
		if (color == SSD1306_COLOR_WHITE) {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x_end + (y_end / 8)*periph_ssd1306->width] |= (1 << (y_end % 8));
		} else {
			periph_ssd1306->buf[periph_ssd1306->buf_idx][x_end + (y_end / 8)*periph_ssd1306->width] &= ~ (1 << (y_end % 8));
		}
	}

	while ((x_start != x_end) || (y_start != y_end))
	{
		if (periph_ssd1306->inverse) {
			if (color == SSD1306_COLOR_WHITE) {
				periph_ssd1306->buf[periph_ssd1306->buf_idx][x_start + (y_start / 8)*periph_ssd1306->width] &= ~ (1 << (y_start % 8));
			} else {
				periph_ssd1306->buf[periph_ssd1306->buf_idx][x_start + (y_start / 8)*periph_ssd1306->width] |= (1 << (y_start % 8));
			}
		} else {
			if (color == SSD1306_COLOR_WHITE) {
				periph_ssd1306->buf[periph_ssd1306->buf_idx][x_start + (y_start / 8)*periph_ssd1306->width] |= (1 << (y_start % 8));
			} else {
				periph_ssd1306->buf[periph_ssd1306->buf_idx][x_start + (y_start / 8)*periph_ssd1306->width] &= ~ (1 << (y_start % 8));
			}
		}

		error2 = error * 2;
		if (error2 > -deltaY)
		{
			error -= deltaY;
			x_start += signX;
		}
		else
		{
			/*nothing to do*/
		}

		if (error2 < deltaX)
		{
			error += deltaX;
			y_start += signY;
		}
		else
		{
			/*nothing to do*/
		}
	}
}

static esp_err_t _update_screen(periph_ssd1306_t *periph_ssd1306)
{
	for (uint8_t i = 0; i < (periph_ssd1306->height / 8); i++) {
		SSD1306_CHECK(!periph_ssd1306->_write_cmd(periph_ssd1306->hw_info, 0xB0 + i), "update screen error", return ESP_FAIL);
		SSD1306_CHECK(!periph_ssd1306->_write_cmd(periph_ssd1306->hw_info, 0x00), "update screen error", return ESP_FAIL);
		SSD1306_CHECK(!periph_ssd1306->_write_cmd(periph_ssd1306->hw_info, 0x10), "update screen error", return ESP_FAIL);
		SSD1306_CHECK(!periph_ssd1306->_write_data(periph_ssd1306->hw_info, &periph_ssd1306->buf[periph_ssd1306->buf_idx][i * periph_ssd1306->width], periph_ssd1306->width), "update screen error", return ESP_FAIL);
	}

	return ESP_OK;
}

static esp_err_t _ssd1306_init(esp_periph_handle_t self)
{
	VALIDATE_SSD1306(self, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(self);
	if (!periph_ssd1306->is_started) {
		periph_ssd1306->is_started = true;
	}

	return ESP_OK;
}

static esp_err_t _ssd1306_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
	return ESP_OK;
}

static esp_err_t _ssd1306_destroy(esp_periph_handle_t self)
{
	VALIDATE_SSD1306(self, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(self);

	mutex_destroy(periph_ssd1306->lock);
	free(periph_ssd1306->buf[0]);
	free(periph_ssd1306->buf[1]);
	free(periph_ssd1306);

	return ESP_OK;
}

esp_periph_handle_t periph_ssd1306_init(periph_ssd1306_cfg_t *config)
{
	SSD1306_CHECK(config, "error config null", return NULL);

	esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_SSD1306, config->tag ? config->tag : "periph_ssd1306");
	periph_ssd1306_t *periph_ssd1306 = calloc(1, sizeof(periph_ssd1306_t));
	PERIPH_MEM_CHECK(TAG, periph_ssd1306, {free(periph); return NULL;});

	uint8_t width = _get_screen_width(config->size);
	uint8_t height = _get_screen_height(config->size);

	periph_ssd1306->buf[0] = calloc(width * height / 8, sizeof(uint8_t));
	SSD1306_CHECK(periph_ssd1306->buf[0], "error allocate buf 0", {_ssd1306_cleanup(periph); return NULL;});

	periph_ssd1306->buf[1] = calloc(width * height / 8, sizeof(uint8_t));
	SSD1306_CHECK(periph_ssd1306->buf[1], "error allocate buf 1", {_ssd1306_cleanup(periph); return NULL;});

	init_func _init = _get_init_func(config->comm_mode);
	SSD1306_CHECK(!_init(config->hw_info), "init error", {_ssd1306_cleanup(periph); return NULL;});

	write_cmd_func _write_cmd = _get_write_cmd_func(config->comm_mode);

	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAY_OFF), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_MEMORYMODE), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_MEMORYMODE_HOR), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_COMSCAN_DEC), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x10), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_STARTLINE_ZERO), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_SEGREMAP_INV), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, config->inverse == false ? SSD1306_DISPLAY_NORMAL : SSD1306_DISPLAY_INVERSE), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xFF), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, config->size == SSD1306_SIZE_128_32 ? 0x1F : 0x3F), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAYALLON_RESUME), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_DISPLAYOFFSET), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_CLKDIV), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xF0), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_PRECHARGE), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x22), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_COMPINS), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, config->size == SSD1306_SIZE_128_32 ? 0x02 : 0x12), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_COMDESELECT), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x20), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_CHARGEPUMP), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_CHARGEPUMP_ON), "init error", {_ssd1306_cleanup(periph); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAY_ON), "init error", {_ssd1306_cleanup(periph); return NULL;});

	periph_ssd1306->hw_info = config->hw_info;
	periph_ssd1306->size = config->size;
	periph_ssd1306->comm_mode = config->comm_mode;
	periph_ssd1306->width = width;
	periph_ssd1306->height = height;
	periph_ssd1306->buf_len = width * height / 8;
	periph_ssd1306->buf_idx = 0;
	periph_ssd1306->_write_cmd = _get_write_cmd_func(config->comm_mode);
	periph_ssd1306->_write_data = _get_write_data_func(config->comm_mode);
	periph_ssd1306->cur_x = 0;
	periph_ssd1306->cur_x = 0;
	periph_ssd1306->inverse = config->inverse;
	periph_ssd1306->is_started = false;
	periph_ssd1306->lock = mutex_create();

	esp_periph_set_data(periph, periph_ssd1306);
	esp_periph_set_function(periph, _ssd1306_init, _ssd1306_run, _ssd1306_destroy);
	g_ssd1306 = periph;

	return periph;
}

esp_err_t periph_ssd1306_clear(esp_periph_handle_t periph)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	for (uint32_t i = 0; i < (periph_ssd1306->width * periph_ssd1306->height / 8); i++) {
		periph_ssd1306->buf[periph_ssd1306->buf_idx][i] = 0x00;
	}
	SSD1306_CHECK(!_update_screen(periph_ssd1306), "clear screen error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_fill(esp_periph_handle_t periph, ssd1306_color_t color)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	for (uint32_t i = 0; i < (periph_ssd1306->width * periph_ssd1306->height / 8); i++) {
		periph_ssd1306->buf[periph_ssd1306->buf_idx][i] = periph_ssd1306->inverse == false ?
		        ((color == SSD1306_COLOR_WHITE) ? 0xFF : 0x00) :
		        ((color == SSD1306_COLOR_WHITE) ? 0x00 : 0xFF);
	}
	SSD1306_CHECK(!_update_screen(periph_ssd1306), "fill screen error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_write_char(esp_periph_handle_t periph, font_size_t font_size, uint8_t chr)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	font_t font;
	SSD1306_CHECK(get_font(chr, font_size, &font) > 0, "get font error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	uint8_t num_byte_per_row = font.data_len / font.height;

	for (uint8_t height_idx = 0; height_idx < font.height; height_idx ++) {
		for ( uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
			for (uint8_t width_idx = 0; width_idx < 8; width_idx++) {
				uint8_t x = periph_ssd1306->cur_x + width_idx + byte_idx * 8;
				uint8_t y = periph_ssd1306->cur_y + height_idx;

				if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
					periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] |= (1 << (y % 8));
				} else {
					periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] &= ~ (1 << (y % 8));
				}
			}
		}
	}

	SSD1306_CHECK(!_update_screen(periph_ssd1306), "write char error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	periph_ssd1306->cur_x += font.width + num_byte_per_row;
	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_write_string(esp_periph_handle_t periph, font_size_t font_size, uint8_t *str)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	SSD1306_CHECK(str, "error string null", return ESP_ERR_INVALID_ARG);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	uint8_t cur_x = periph_ssd1306->cur_x;
	uint8_t cur_y = periph_ssd1306->cur_y;

	while (*str) {
		font_t font;
		SSD1306_CHECK(get_font(*str, font_size, &font) > 0, "get font error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

		uint8_t num_byte_per_row = font.data_len / font.height;

		for (uint8_t height_idx = 0; height_idx < font.height; height_idx ++) {
			for ( uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
				for (uint8_t width_idx = 0; width_idx < 8; width_idx++) {
					uint8_t x = cur_x + width_idx + byte_idx * 8;
					uint8_t y = cur_y + height_idx;

					if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
						periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] |= (1 << (y % 8));
					} else {
						periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] &= ~ (1 << (y % 8));
					}
				}
			}
		}
		cur_x += font.width + num_byte_per_row;
		str++;
	}

	SSD1306_CHECK(!_update_screen(periph_ssd1306), "write string error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	periph_ssd1306->cur_x = cur_x;
	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_draw_pixel(esp_periph_handle_t periph, uint8_t x, uint8_t y, ssd1306_color_t color)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	_draw_pixel(periph_ssd1306, x, y, color);
	SSD1306_CHECK(!_update_screen(periph_ssd1306), "draw pixel error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_draw_line(esp_periph_handle_t periph, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	_draw_line(periph_ssd1306, x1, y1, x2, y2, color);

	SSD1306_CHECK(!_update_screen(periph_ssd1306), "draw line error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_draw_rectangle(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, ssd1306_color_t color)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	_draw_line(periph_ssd1306, x_origin, y_origin, x_origin + width, y_origin, color);
	_draw_line(periph_ssd1306, x_origin + width, y_origin, x_origin + width, y_origin + height, color);
	_draw_line(periph_ssd1306, x_origin + width, y_origin + height, x_origin, y_origin + height, color);
	_draw_line(periph_ssd1306, x_origin, y_origin + height, x_origin, y_origin, color);

	SSD1306_CHECK(!_update_screen(periph_ssd1306), "draw rectangle error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_draw_circle(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t radius, ssd1306_color_t color)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	int32_t x = -radius;
	int32_t y = 0;
	int32_t err = 2 - 2 * radius;
	int32_t e2;

	do {
		_draw_pixel(periph_ssd1306, x_origin - x, y_origin + y, color);
		_draw_pixel(periph_ssd1306, x_origin + x, y_origin + y, color);
		_draw_pixel(periph_ssd1306, x_origin + x, y_origin - y, color);
		_draw_pixel(periph_ssd1306, x_origin - x, y_origin - y, color);

		e2 = err;
		if (e2 <= y) {
			y++;
			err = err + (y * 2 + 1);
			if (-x == y && e2 <= x) {
				e2 = 0;
			}
			else {
				/*nothing to do*/
			}
		} else {
			/*nothing to do*/
		}

		if (e2 > x) {
			x++;
			err = err + (x * 2 + 1);
		} else {
			/*nothing to do*/
		}
	} while (x <= 0);
	SSD1306_CHECK(!_update_screen(periph_ssd1306), "draw circle error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_draw_image(esp_periph_handle_t periph, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, uint8_t *img)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	mutex_lock(periph_ssd1306->lock);

	periph_ssd1306->buf_idx ^= 1;
	memcpy(periph_ssd1306->buf[periph_ssd1306->buf_idx], periph_ssd1306->buf[periph_ssd1306->buf_idx ^ 1], periph_ssd1306->buf_len);

	uint8_t num_byte_per_row = width / 8;

	for (uint8_t height_idx = 0; height_idx < height; height_idx++) {
		for (uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
			for (uint8_t width_idx = 0; width_idx < 8; width_idx++) {
				uint8_t x = width_idx + byte_idx * 8;
				uint8_t y = height_idx;

				if (((img[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
					periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] |= (1 << (y % 8));
				} else {
					periph_ssd1306->buf[periph_ssd1306->buf_idx][x + (y / 8)*periph_ssd1306->width] &= ~ (1 << (y % 8));
				}
			}
		}
	}
	SSD1306_CHECK(!_update_screen(periph_ssd1306), "draw image error", {mutex_unlock(periph_ssd1306->lock); return ESP_FAIL;});

	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_set_position(esp_periph_handle_t periph, uint8_t x, uint8_t y)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	SSD1306_CHECK(x < periph_ssd1306->width, "error position", return ESP_ERR_INVALID_ARG);
	SSD1306_CHECK(y < periph_ssd1306->height, "error position", return ESP_ERR_INVALID_ARG);

	mutex_lock(periph_ssd1306->lock);
	periph_ssd1306->cur_x = x;
	periph_ssd1306->cur_y = y;
	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}

esp_err_t periph_ssd1306_get_position(esp_periph_handle_t periph, uint8_t *x, uint8_t *y)
{
	VALIDATE_SSD1306(periph, ESP_FAIL);
	periph_ssd1306_t *periph_ssd1306 = esp_periph_get_data(periph);

	SSD1306_CHECK(x, "error position null", return ESP_ERR_INVALID_ARG);
	SSD1306_CHECK(y, "error position null", return ESP_ERR_INVALID_ARG);

	mutex_lock(periph_ssd1306->lock);
	*x = periph_ssd1306->cur_x;
	*y = periph_ssd1306->cur_y;
	mutex_unlock(periph_ssd1306->lock);

	return ESP_OK;
}