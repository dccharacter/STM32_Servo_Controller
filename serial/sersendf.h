#ifndef	_SERSENDF_H
#define	_SERSENDF_H

<<<<<<< HEAD
//#include	<avr/pgmspace.h>

void sersendf(char *format, ...);//		__attribute__ ((format (printf, 1, 2)));
//void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));
=======
#include	<avr/pgmspace.h>

void sersendf(char *format, ...)		__attribute__ ((format (printf, 1, 2)));
void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));
>>>>>>> Using original teacup firmware files

#endif	/* _SERSENDF_H */
