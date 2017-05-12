#include <stdint.h>
#include <stdbool.h>
#include "USI_TWI_Master/USI_TWI_Master.h"

void i2c_init();
bool i2c_read(uint8_t slave_address, uint8_t byte_address, uint8_t *buffer, uint8_t size);
bool i2c_write(uint8_t slave_address, uint8_t byte_address, uint8_t *buffer, uint8_t size);
bool i2c_detect(uint8_t slave_address);

