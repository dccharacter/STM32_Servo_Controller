#ifndef	_TEMP_H
#define	_TEMP_H

<<<<<<< HEAD
//#include	"config.h"
=======
#include	"config.h"
>>>>>>> Using original teacup firmware files
#include	<stdint.h>

/*
NOTES

no point in specifying a port- all the different temp sensors we have must be on a particular port. The MAX6675 must be on the SPI, and the thermistor and AD595 must be on an analog port.

we still need to specify which analog pins we use in machine.h for the analog sensors however, otherwise the analog subsystem won't read them.
*/

<<<<<<< HEAD
/*#undef DEFINE_TEMP_SENSOR
=======
#undef DEFINE_TEMP_SENSOR
>>>>>>> Using original teacup firmware files
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) TEMP_SENSOR_ ## name,
typedef enum {
	#include "config.h"
	NUM_TEMP_SENSORS,
	TEMP_SENSOR_none
} temp_sensor_t;
<<<<<<< HEAD
#undef DEFINE_TEMP_SENSOR*/
=======
#undef DEFINE_TEMP_SENSOR
>>>>>>> Using original teacup firmware files

typedef enum {
	TT_THERMISTOR,
	TT_MAX6675,
	TT_AD595,
	TT_PT100,
	TT_INTERCOM,
	TT_NONE,
	TT_DUMMY,
} temp_type_t;

#define	temp_tick temp_sensor_tick

void temp_init(void);

void temp_sensor_tick(void);

uint8_t	temp_achieved(void);

<<<<<<< HEAD
//void temp_set(temp_sensor_t index, uint16_t temperature);
//uint16_t temp_get(temp_sensor_t index);

uint8_t temp_all_zero(void);

void temp_print(void);
=======
void temp_set(temp_sensor_t index, uint16_t temperature);
uint16_t temp_get(temp_sensor_t index);

uint8_t temp_all_zero(void);

void temp_print(temp_sensor_t index);
>>>>>>> Using original teacup firmware files

#endif	/* _TEMP_H */
