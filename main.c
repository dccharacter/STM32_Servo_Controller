#include "./hrdw_cfg/hrdw_cfg.h"
#include <stdio.h>

int main(void)
{
	Hardware_Configuration();
	printf("\fHi!%c\r\n", ' ');

    while(1)
    {
    }
}
