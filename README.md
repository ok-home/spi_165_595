```
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
```