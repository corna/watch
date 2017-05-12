#include <i2c.h>

bool veml7700_set(uint8_t address, bool power);
bool veml7700_get(uint8_t address, uint16_t *data);
