#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "gps.h"
#include "NRF_driver.h"
#include "dGPS.h"
#include "GPS_Route_Setter.h"

#define dGPS_calculator_debug

dGPS_decimalData_t latestCorrectedGPS; // Holds the latest corrected GPS data

void GPS_getlatest_corrected(dGPS_decimalData_t *dest)
{
    /* Copy the latest corrected GPS data safely */
    if(xSemaphoreTake(hdGPSlatest_Mutex, portMAX_DELAY) == pdTRUE)
    {
        memcpy(dest, &latestCorrectedGPS, sizeof(dGPS_decimalData_t));
        xSemaphoreGive(hdGPSlatest_Mutex);
    }
    else
    {
        error_HaltOS("Err:hdGPSlatest_Mutex");
    }
}

/**
 * @brief Compares the latest GPS data with the latest error data, gives a latest valid GPS datapoint using the received time from the error data. 
 * This function should only be called upon receiving
 * new error data from the NRF module.
 * 
 */
void dGPS_comparator()
{
    /*Get latest GPS buffer and latest error, and initialize pointers to them*/
    dGPS_decimalData_t latestGPSbuffer[60];
    dGPS_errorData_t receivedError;

    PdGPS_decimalData_t ptdGPS = &latestGPSbuffer[0];
    PdGPS_errorData_t ptdError = &receivedError;

    GPS_getlatest_ringbuffer(latestGPSbuffer);
    GPS_getlatest_error(&receivedError);

    /* Search the copied buffer for a matching timestamp.*/
    PdGPS_decimalData_t end = &latestGPSbuffer[60]; /* one past the last element */

    for (; ptdGPS < end; ++ptdGPS)
    {
        if (ptdGPS->timestamp == ptdError->timestamp)
            break; /* found */
    }

    /* If a matching timestamp is found, correct the coordinates and update the frontend buffer */
    if (ptdGPS != end)
    {
        UART_puts("     Matching timestamp found in GPS buffer for received error data\r\n");

        /* Correct for received GPS error and store result under mutex */
        if (xSemaphoreTake(hdGPSlatest_Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            latestCorrectedGPS.latitude  = ptdGPS->latitude - ptdError->latitude;
            latestCorrectedGPS.longitude = ptdGPS->longitude - ptdError->longitude;
            latestCorrectedGPS.timestamp = ptdGPS->timestamp;
            xSemaphoreGive(hdGPSlatest_Mutex);

            #ifdef dGPS_calculator_debug
                char msg[150];
                sprintf(msg, "      Corrected GPS stored: Time: %ld Lat: %.9f, Lon: %.9f\r\n", latestCorrectedGPS.timestamp, latestCorrectedGPS.latitude, latestCorrectedGPS.longitude);
                UART_puts(msg);
            #endif
        }
        else
        {
            /* If mutex cannot be taken quickly, still return; caller may retry later */
            UART_puts("         Warning: could not take hdGPSlatest_Mutex to store corrected GPS\r\n");
        }

        return;
    }

    if (ptdGPS == end)
    {
        /* No matching timestamp found in the buffer */
        #ifdef dGPS_calculator_debug
            char msg[150];
            snprintf(msg, sizeof(msg), "        No matching time found for timestamp: %ld in GPS error buffer\r\n", ptdError->timestamp);
            UART_puts(msg);
        #endif
        return; /* No matching timestamp found */
    }
}

void dGPS_calculator(void *argument)
{
    osDelay(100);
    UART_puts((char *)__func__); UART_puts(" started\r\n");

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification from NRF driver task
        osDelay(100);
        UART_puts("\r\n     dGPS_calculator notified, new error received\r\n");
        /* Process the newly received error by searching the GPS history and computing corrected coords */
        dGPS_comparator();
        osDelay(1);
    }
}

