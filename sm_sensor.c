#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/adc.h"


#define UART_PORT_NUM      UART_NUM_0  
#define UART_BAUD_RATE     115200
#define UART_BUFFER_SIZE   1024

#define LED_GPIO_PIN       2
#define SENSOR_PIN ADC1_CHANNEL_6


static QueueHandle_t uart_queue;
int moisture,sensorAnalog;

void configure_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);  // ADC veri genişliğini 12 bit olarak ayarla
    adc1_config_channel_atten(SENSOR_PIN, ADC_ATTEN_DB_11);  // Kanal zayıflatmasını ayarla
}

int read_sensor_value() {
    int value = (adc1_get_raw(SENSOR_PIN));
    moisture = 100-(100 * (3022.00 - value) / 3022.00);
    return moisture;
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

                        if (strncmp((char *) data, "Led_on", strlen("Led_on")) == 0) {
                            gpio_set_level(LED_GPIO_PIN, 1);
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            printf("LED yandi\n");

                        } else if (strncmp((char *) data, "Led_off", strlen("Led_off")) == 0) {
                            gpio_set_level(LED_GPIO_PIN, 0);
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            printf("LED sondu\n");

                        } else if (strncmp((char *) data, "ms_sensor", strlen("ms_sensor")) == 0){
                                sensorAnalog = read_sensor_value();
                                printf("Sensor degeri: %d", sensorAnalog);
                                vTaskDelay(10 / portTICK_PERIOD_MS);
                                int num = adc1_get_raw(SENSOR_PIN);
                                printf("deger: %d\n", num);
                                if(sensorAnalog<50){
                                    gpio_set_level(LED_GPIO_PIN, 1);
                                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                                    printf("Nem cok az. Sulama gerekli.\n");
                                    gpio_set_level(LED_GPIO_PIN, 0);
                                }
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

    // UART olay işleme görevini başlat
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
}
