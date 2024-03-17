#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include <string.h>

#define SPI_HOST 2
#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK 7
#define PIN_NUM_CS 8


int8_t spi_read(spi_device_handle_t sensor_handle,uint16_t cmd, uint16_t *data)
{

    int8_t error_status;
    spi_transaction_t spi_transaction;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    //spi_transaction.cmd = cmd;
    spi_transaction.length = 16; 
    spi_transaction.rxlength = 16; 
    spi_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    //spi_transaction.tx_data = command;
    *((uint16_t*)spi_transaction.tx_data) = SPI_SWAP_DATA_TX(cmd, 16);
    //spi_transaction.tx_buffer = NULL;
    // We send one transaction, which is our READ comment. We don't care about the received part yet. 
    spi_device_acquire_bus(sensor_handle, portMAX_DELAY);
    error_status = spi_device_transmit(sensor_handle, &spi_transaction);
    spi_device_release_bus(sensor_handle);

    if (error_status != ESP_OK) {
        ESP_LOGE("TAG", "Error sending read command to register: %d", cmd);
        return -1; // Return -1 on failure
    }

    // We send another (arbitrary) command, because what we now are interested
    // in is the received data, which is the response to the above read command. 
    spi_device_acquire_bus(sensor_handle, portMAX_DELAY);
    error_status = spi_device_transmit(sensor_handle, &spi_transaction);
    spi_device_release_bus(sensor_handle);
    if (error_status != ESP_OK) {
        ESP_LOGE("TAG", "Error receiving read data from register.");
        return -1; // Return -1 on failure
    }
    
    *data = SPI_SWAP_DATA_RX(*((uint16_t*)spi_transaction.rx_data), 16); 
   // response is always 0110000000000000;
  
    return error_status; 
}

    #include "logic_analyzer_ws_server.h"

void app_main(void)
{
    logic_analyzer_ws_server();

    int8_t error_status;

    // First, configure the bus
    spi_bus_config_t spi_bus_config = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16
    };

    // Now initialize the SPI bus
    error_status = spi_bus_initialize(SPI_HOST, &spi_bus_config, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(error_status);

    spi_device_interface_config_t sensor_config = {
        .clock_speed_hz = 300 * 1000, 
        .mode = 1,                              
        .spics_io_num = PIN_NUM_CS, 
        .queue_size = 1,                     
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .input_delay_ns = 350,
        .flags = SPI_DEVICE_3WIRE,
//        .cs_ena_pretrans = 2,
//        .cs_ena_posttrans = 2
    };

    // Attach the sensor
    spi_device_handle_t sensor_handle;
    error_status = spi_bus_add_device(SPI_HOST, &sensor_config, &sensor_handle);
    ESP_ERROR_CHECK(error_status);

    uint16_t data;
    uint16_t cmd=0;
    while(1){
    spi_read(sensor_handle,(cmd++),&data);
    ESP_LOGI("SPI","data=%x",data);
    vTaskDelay(100);
    }

}

