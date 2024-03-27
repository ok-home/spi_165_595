idf_component_register(SRCS "main.c"
                    REQUIRES 
                        logic_analyzer 
                        driver
                        spi_165_595
                    INCLUDE_DIRS ".")