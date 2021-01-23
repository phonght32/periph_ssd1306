#include "stdio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "periph_ssd1306.h"

uint8_t pikachu_map[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x2f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x2f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf8, 0x4f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfd, 0xbf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf3, 0x1f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x46, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf7, 0x99, 0x9f, 0x8a, 0x6f,
    0xff, 0xff, 0xff, 0xfb, 0xbf, 0x57, 0x6e, 0xff,
    0xff, 0xff, 0xff, 0xf5, 0xff, 0xdf, 0xbf, 0x80,
    0xff, 0xff, 0xa7, 0xf5, 0xff, 0xef, 0xfe, 0x29,
    0xff, 0xf8, 0xeb, 0xef, 0xff, 0xfb, 0xe8, 0x1f,
    0xff, 0xd8, 0xeb, 0xef, 0xff, 0xfa, 0xd4, 0xbf,
    0xfe, 0x61, 0xff, 0xe5, 0xbf, 0xfd, 0xbf, 0xff,
    0xfb, 0x4f, 0xe6, 0xcc, 0x7f, 0xff, 0xff, 0xff,
    0xec, 0x7f, 0xc4, 0x6c, 0xff, 0xfc, 0xff, 0xff,
    0xe3, 0xff, 0xc8, 0x9d, 0xff, 0xb7, 0xff, 0xff,
    0xef, 0xff, 0xde, 0x6e, 0x3f, 0xaf, 0xff, 0xff,
    0xff, 0xff, 0xc7, 0x9b, 0xd9, 0xbb, 0xff, 0xff,
    0xbf, 0xff, 0x87, 0xd9, 0xf4, 0xb3, 0xff, 0xff,
    0xbf, 0xff, 0x97, 0x85, 0xfd, 0x19, 0xff, 0xff,
    0x9f, 0xf5, 0xff, 0xdf, 0xdb, 0xab, 0x7f, 0xff,
    0xdf, 0xcd, 0x43, 0xc3, 0xdf, 0x74, 0x77, 0xff,
    0x9f, 0xb5, 0xc7, 0xef, 0xfc, 0xfb, 0xfb, 0xff,
    0xce, 0x37, 0x4f, 0xe3, 0xff, 0xd5, 0xf7, 0xff,
    0xce, 0x7f, 0xb1, 0xff, 0xff, 0xfb, 0xcf, 0xff,
    0xcf, 0xfb, 0xf5, 0xff, 0xe7, 0x9f, 0xef, 0xff,
    0xf5, 0xff, 0xfb, 0xff, 0xef, 0x1f, 0xcf, 0xff,
    0xf7, 0xe6, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xf4, 0xa3, 0x7f, 0xff, 0xf8, 0xff, 0xff,
    0xff, 0xe9, 0x49, 0xff, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xf7, 0x35, 0xff, 0xff, 0xbf, 0xff, 0xff,
    0xff, 0xef, 0xa9, 0xff, 0xff, 0xcf, 0xff, 0xff,
    0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0x7f, 0xff, 0xff, 0x5f, 0xff, 0xff,
    0xff, 0xff, 0xef, 0xff, 0xff, 0xdf, 0xff, 0xff,
    0xff, 0xff, 0xf7, 0xff, 0xff, 0xef, 0xff, 0xff,
    0xff, 0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff,
    0xff, 0xff, 0xcf, 0xff, 0xff, 0xf7, 0xff, 0xff,
    0xff, 0xff, 0x9f, 0xff, 0xff, 0xeb, 0xff, 0xff,
    0xff, 0xff, 0xbf, 0xff, 0xff, 0xeb, 0xff, 0xff,
    0xff, 0xff, 0x9f, 0xff, 0xff, 0xf3, 0xff, 0xff,
    0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff,
    0xff, 0xff, 0xa0, 0x1f, 0x9f, 0xf7, 0xff, 0xff,
    0xff, 0xff, 0x49, 0xf9, 0x89, 0xf7, 0xff, 0xff,
    0xff, 0xfe, 0x13, 0xdf, 0xc8, 0x39, 0xff, 0xff,
    0xff, 0xff, 0xa3, 0xff, 0xfe, 0x37, 0xff, 0xff,
    0xff, 0xff, 0x4f, 0xff, 0xff, 0xf3, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

esp_periph_handle_t ssd1306_handle;
static i2cdev_handle_t i2cdev = NULL;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{
    esp_periph_config_t config = {
        .event_handle = periph_event_handle,
        .max_parallel_connections = 9,
    };
    esp_periph_init(&config);

    i2cdev_cfg_t i2cdev_cfg = {
        .port = I2C_NUM_1,
        .mode  = I2C_MODE_MASTER,
        .sda_io_num = 5,
        .scl_io_num = 4,
        .clk_speed = 100000,
    };
    i2cdev = i2cdev_init(&i2cdev_cfg);

    ssd1306_hw_info_t ssd1306_hw_info = {
        .i2cdev = i2cdev,
    };

    periph_ssd1306_cfg_t periph_ssd1306_cfg = {
        .hw_info = ssd1306_hw_info,
        .size = SSD1306_SIZE_128_64,
        .comm_mode = SSD1306_COMM_MODE_I2C,
        .inverse = false,
    };

    ssd1306_handle = periph_ssd1306_init(&periph_ssd1306_cfg);
    esp_periph_start(ssd1306_handle);

    periph_ssd1306_clear(ssd1306_handle);
    periph_ssd1306_draw_image(ssd1306_handle, 0, 0, 64, 64, pikachu_map);
    periph_ssd1306_set_position(ssd1306_handle, 80, 25);
    periph_ssd1306_write_string(ssd1306_handle, FONT_SIZE_5x8, (uint8_t *)"EXAMPLE");
    periph_ssd1306_set_position(ssd1306_handle, 70, 40);
    periph_ssd1306_write_string(ssd1306_handle, FONT_SIZE_5x8, (uint8_t *)"Oled 128x64");

    while (1)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
