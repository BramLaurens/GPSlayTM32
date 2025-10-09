#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "gps.h"
#include "NRF_driver.h"
#include "dGPS.h"
#include "GPS_Route_Setter.h"

#define dGPS_calculator_debug

void dGPS_calculator(void *argument)
{
    osDelay(100);
    UART_puts((char *)__func__); UART_puts(" started\r\n");

    while (1)
    {
        // Placeholder delay, replace with actual processing logic
        osDelay(1);
    }
}

