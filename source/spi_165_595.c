/* spi out to 595 in from 165

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include <string.h>

#define SPI_NUM 2
#define SPI_CLK (100 * 1000)

static gpio_num_t spi_latch_load_pin = -1;
//
// read 165 ( 8 bit ) after write 959 (16 bit)
// data latched to parallel output 595 and load to parallel input 165 on positive edge spi_latch_load_pin
//
esp_err_t spi_165_595_write_read(spi_device_handle_t spi_165_595_handle, uint16_t data_out, uint8_t *data_in)
{
    esp_err_t error_status;
    spi_transaction_t spi_transaction;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction.length = 16;
    spi_transaction.rxlength = 0;
    spi_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    *((uint16_t *)spi_transaction.tx_data) = SPI_SWAP_DATA_TX(data_out, 16);
    // shift data_out to 595
    gpio_set_level(spi_latch_load_pin, 0);
    error_status = spi_device_transmit(spi_165_595_handle, &spi_transaction);
    // latch data_out to parallel out 595
    // load data_in from parallel to shift 165
    gpio_set_level(spi_latch_load_pin, 1);
    if (error_status != ESP_OK)
    {
        ESP_LOGE("TAG", "Error sending data to register: %d", data_out);
        return -1; // Return -1 on failure
    }
    //*((uint16_t *)spi_transaction.tx_data) = SPI_SWAP_DATA_TX(data_out, 16);
    // set serial dout level to 1 ( data not latched to parallel out 595)
    *((uint16_t *)spi_transaction.tx_data) = SPI_SWAP_DATA_TX(0xff, 8);
    spi_transaction.length = 8;
    spi_transaction.rxlength = 8;
    // shift loaded data from 165 to data_in
    error_status = spi_device_transmit(spi_165_595_handle, &spi_transaction);
    if (error_status != ESP_OK)
    {
        ESP_LOGE("TAG", "Error receiving data from register.");
        return -1; // Return -1 on failure
    }
    *data_in = SPI_SWAP_DATA_RX(*((uint16_t *)spi_transaction.rx_data), 8);

    return error_status;
}
// init SPI
// SPI_NUM & SPI_CLK predefined
// in/out/clk/latch_lock pin as input argument
// return spi handle

spi_device_handle_t spi_165_595_init(gpio_num_t sr_out_pin, gpio_num_t sr_in_pin, gpio_num_t sr_clk_pin, gpio_num_t latch_load_pin)
{
    esp_err_t error_status;
    // First, configure the bus
    spi_bus_config_t spi_bus_config = {
        .miso_io_num = sr_in_pin,
        .mosi_io_num = sr_out_pin,
        .sclk_io_num = sr_clk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16};

    // Now initialize the SPI bus
    error_status = spi_bus_initialize(SPI_NUM, &spi_bus_config, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(error_status);

    spi_device_interface_config_t spi_interface_cfg = {
        .clock_speed_hz = SPI_CLK,
        .mode = 3,                  // mode=0,1,2,3 level and edge of sclk
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .input_delay_ns = 200,
    };

    // Attach the 165_595
    spi_device_handle_t spi_165_595_handle;
    error_status = spi_bus_add_device(SPI_NUM, &spi_interface_cfg, &spi_165_595_handle);
    ESP_ERROR_CHECK(error_status);
    // init latch_load pin
    gpio_reset_pin(latch_load_pin);
    gpio_set_direction(latch_load_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(latch_load_pin, 1);
    spi_latch_load_pin = latch_load_pin;

    return spi_165_595_handle;
}
