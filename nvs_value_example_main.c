#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "nvs.h"
#include "nvs_flash.h"

#define UART_PORT_NUM      UART_NUM_0  
#define UART_BAUD_RATE     115200
#define UART_BUFFER_SIZE   1024

#define LED_GPIO_PIN       2

static QueueHandle_t uart_queue;

void save_led_state(uint8_t state) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
   
    if (err == ESP_OK) {
      
        err = nvs_set_u8(nvs_handle, "led_state", state);
        if (err == ESP_OK) {

            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    if (err != ESP_OK) {
        printf("Error saving LED state: %s\n", esp_err_to_name(err));
    }
}

uint8_t get_led_state() {
    nvs_handle_t nvs_handle;
    uint8_t state = 0;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_get_u8(nvs_handle, "led_state", &state);
        nvs_close(nvs_handle);
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error getting LED state: %s\n", esp_err_to_name(err));
    }
    return state;
}

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[UART_BUFFER_SIZE];
    int len;

    for (;;) {
        if (xQueueReceive(uart_queue, (void *) &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
                    if (len > 0) {
                        data[len] = '\0'; 
                        printf("Received: %s\n", data);

                        if (strncmp((char *) data, "Led_yak", strlen("Led_yak")) == 0) {
                            gpio_set_level(LED_GPIO_PIN, 1);
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            printf("LED yandi\n");
                            save_led_state(1);
                        } else if (strncmp((char *) data, "Led_sondur", strlen("Led_sondur")) == 0) {
                            gpio_set_level(LED_GPIO_PIN, 0);
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            printf("LED sondu\n");
                            save_led_state(0);
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret == ESP_OK) {
            ret = nvs_flash_init();  
        }
    }

    if (ret != ESP_OK) {
        printf("NVS init/erase failed with error: %s\n", esp_err_to_name(ret));
        
    }

    // UART konfigürasyonu
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // UART yapılandırması
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, (unsigned int) UART_BUFFER_SIZE * 2, (unsigned int) UART_BUFFER_SIZE * 2, 10, &uart_queue, 0);

    // LED GPIO konfigürasyonu
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Retrieve and set the last LED state
    uint8_t led_state = get_led_state();
    gpio_set_level(LED_GPIO_PIN, led_state);

    // UART olay işleme görevini başlat
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
}
