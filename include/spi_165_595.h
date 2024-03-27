/* spi out to 595 in from 165

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// init SPI
// SPI_NUM & SPI_CLK predefined
// in/out/clk/latch_lock pin as input argument 
// return spi handle
spi_device_handle_t spi_165_595_init(gpio_num_t sr_out_pin, gpio_num_t sr_in_pin, gpio_num_t sr_clk_pin, gpio_num_t latch_load_pin);
//
// read 165 ( 8 bit ) after write 959 (16 bit)
// data latched to parallel output 595 and load to parallel input 165 on positive edge spi_latch_load_pin
// 
esp_err_t spi_165_595_write_read(spi_device_handle_t spi_165_595_handle,uint16_t data_out, uint8_t *data_in);
