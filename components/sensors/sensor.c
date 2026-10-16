#include "lbr/sensor.h"

/*
 * No concrete sensor drivers yet (parts TBD). Each driver provides a function
 * that returns a populated lbr_sensor_t for its node to register. Keep bus
 * access (I2C/SPI/analog) inside the driver, behind this interface.
 */
