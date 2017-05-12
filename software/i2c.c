#include "i2c.h"

#define I2C_SLAVE_WRITE(x) (x & 0xfe)
#define I2C_SLAVE_READ(x) (x | 0x01)

void i2c_init()
{
	USI_TWI_Master_Initialise();
}

bool i2c_read(uint8_t slave_address, uint8_t byte_address, uint8_t *buffer, uint8_t size)
{
	slave_address = I2C_SLAVE_WRITE(slave_address);
	if (USI_TWI_Start_Transceiver_With_Data_Stop(&slave_address, 1, false)) {
		if (USI_TWI_Start_Transceiver_With_Data_Stop(&byte_address, 1, false)) {
			slave_address = I2C_SLAVE_READ(slave_address);
			if (USI_TWI_Start_Transceiver_With_Data_Stop(&slave_address, 1, false))
				if (USI_TWI_Start_Transceiver_With_Data_Stop(buffer, size, true))
					return true;
		}
	}
	
	return false;
}

bool i2c_write(uint8_t slave_address, uint8_t byte_address, uint8_t *buffer, uint8_t size)
{
	slave_address = I2C_SLAVE_WRITE(slave_address);
	if (USI_TWI_Start_Transceiver_With_Data_Stop(&slave_address, 1, false))
		if (USI_TWI_Start_Transceiver_With_Data_Stop(&byte_address, 1, false))
			if (USI_TWI_Start_Transceiver_With_Data_Stop(buffer, size, true))
				return true;

	return false;	
}

bool i2c_detect(uint8_t slave_address)
{
	slave_address = I2C_SLAVE_READ(slave_address);
	return USI_TWI_Start_Transceiver_With_Data_Stop(&slave_address, 1, true);
}

