#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include <string.h>
#include "spi_165_595.h"

#define PIN_NUM_MISO 22
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 19

    #include "logic_analyzer_ws_server.h"

void app_main(void)
{
    logic_analyzer_ws_server();

    uint8_t data_in;
    uint16_t data_out=0;
    spi_device_handle_t spi_165_595_handle;
    spi_165_595_handle = spi_165_595_init(PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS);    
    while(1){
    spi_165_595_write_read(spi_165_595_handle,data_out++,&data_in);
    ESP_LOGI("SPI","data=%x",data_in);
    vTaskDelay(100);
    }

}

