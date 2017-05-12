#include "veml7700.h"

bool veml7700_set(uint8_t address, bool power)
{
	//Reg 0: 0001 0010 0000 000(0/1)	Power (on/off), x(1/8) sensitivity, 50 ms integration, no interrupt
	//Reg 3: 0000 0000 0000 0111		4200 ms refresh time, power save enabled
	uint8_t data[2];

	if (power)
	{
		data[0] = 0x07;		//TODO little or big endian?
		data[1] = 0x00;
		if (!i2c_write(address, 0x03, data, 2))
			return false;

		data[0] = 0x00;
	}
	else
		data[0] = 0x01;

	data[1] = 0x12;

	if (!i2c_write(address, 0x01, data, 2))
		return false;
	else
		return true;
}

bool veml7700_get(uint8_t address, uint16_t *data)
{
	uint8_t temp[2];

	if (i2c_read(address, 0x04, temp, 2))
	{
		*data = (uint16_t)temp[1] << 8 | temp[1];
		return true;
	}
	else
		return false;
}
