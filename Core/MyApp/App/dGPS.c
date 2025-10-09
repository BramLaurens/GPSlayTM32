#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "gps.h"
#include "NRF_driver.h"
#include "dGPS.h"
#include "GPS_Route_Setter.h"

#define dGPS_debug

GNRMC gnrmc_localbuffer; // local struct for GNRMC-messages

dGPS_decimalData_t GPS_history[60]; // Circular buffer to hold last 60 seconds of GPS data

GPS_decimal_degrees_t GPS_workingavgbuffer; // Buffer to hold the current average GPS data

dGPS_decimalData_t GPS_latest_averaged; // Holds the latest averaged GPS data for adding to the history buffer

char last_GPS_time[10] = "000000.000"; // To track time changes
char c_received_GPS_time[10] = "000000.000"; // Current received GPS time
uint32_t u_last_GPS_time = 0;
uint32_t u_working_GPS_time = 0; // Working time variable

int i = 0; // Index for averaging counter

/**
 * @brief Function to store the latest averaged GPS data into the circular history buffer
 * 
 * @param time Current GPS time in HHMMSS format, used to determine the index in the circular buffer
 */
void GPS_store_in_history(uint32_t time)
{
    int index = time % 100; // Circular buffer index (0-59)
    GPS_history[index] = GPS_latest_averaged; // Store the latest averaged data

    #ifdef dGPS_debug
        char msg[100];
        sprintf(msg, "Stored averaged coordinate in history at index %d: Time: %ld Lat: %.9f, Lon: %.9f\r\n", index, time, GPS_history[index].latitude, GPS_history[index].longitude);
        UART_puts(msg);
    #endif
}

void AVG_gpscalc(uint32_t time)
{
    GPS_workingavgbuffer.latitude /= i;
    GPS_workingavgbuffer.longitude /= i;
    GPS_latest_averaged.latitude = GPS_workingavgbuffer.latitude;
    GPS_latest_averaged.longitude = GPS_workingavgbuffer.longitude;
    GPS_latest_averaged.timestamp = time;
}

/**
 * @brief Add GPS data to the averaging buffer.
 * 
 * @param i Number of data points added so far in the current second
 * @param time Current GPS time in HHMMSS format
 */
void AVG_gpsadddata(int i, uint32_t time)
{
    // Convert NMEA coordinates to decimal degrees and add to working average buffer
    double lat_dd = convert_decimal_degrees(gnrmc_localbuffer.latitude, &gnrmc_localbuffer.NS_ind);
    double lon_dd = convert_decimal_degrees(gnrmc_localbuffer.longitude, &gnrmc_localbuffer.EW_ind);
    GPS_workingavgbuffer.latitude += lat_dd;
    GPS_workingavgbuffer.longitude += lon_dd;

    #ifdef dGPS_debug
        char msg[100];
        sprintf(msg, "Adding data to averaging buffer %d: for time: %ld Lat: %.9f, Lon: %.9f\r\n", i, time, lat_dd, lon_dd);
        UART_puts(msg);
    #endif
}

/**
 * @brief Initialize the GPS averaging buffer
 * 
 */
void AVG_gpsinit()
{
    GPS_workingavgbuffer.latitude = 0.0;
    GPS_workingavgbuffer.longitude = 0.0;
}

/**
 * @brief Parse the received GNRMC data and manage the averaging process. Also adds the averaged data to the history buffer to be used later for error matching
 * 
 */
void parse_GPSdata()
{
    // gnrmc_localbuffer is expected to already contain the GNRMC to process
    if(gnrmc_localbuffer.status != 'A') // If status is not valid, skip processing
    {
        #ifdef dGPS_debug
            UART_puts("Invalid GPS data received (status N). Skipping error calculation.\r\n");
        #endif
        return;
    }

    // Extract time and convert to integer seconds in HHMMSS format
    GNRMC *ptd = &gnrmc_localbuffer;
    char *s;

    strcpy(c_received_GPS_time, ptd->time);
    s = strtok(c_received_GPS_time, "."); // Split at decimal point
    u_working_GPS_time = atoi(s); // Convert to integer (hhmmss)

    // Check if the time has changed (new second), then we start a new averaging cycle
    if (u_working_GPS_time != u_last_GPS_time)
    {
        /* New second, reset the averaging buffer and counter*/
        #ifdef dGPS_debug
            UART_puts("\r\nNew second detected. Resetting averaging buffer.\r\n");
        #endif
        i=0;
        AVG_gpsinit();
        AVG_gpsadddata(i+1, u_working_GPS_time);
        u_last_GPS_time = u_working_GPS_time;
        i++;
    }
    else
    {
        AVG_gpsadddata(i+1, u_working_GPS_time);
        i++;
    }

    // If we have collected 5 data points, calculate the average and store it in the circular buffer
    if (i >= 5)
    {
        AVG_gpscalc(u_working_GPS_time);
        GPS_store_in_history(u_working_GPS_time);
    }
}

void dGPS_parser(void *argument)
{
    osDelay(100);
    UART_puts((char *)__func__); UART_puts(" started\r\n");

    while(TRUE)
    {
        // Receive parsed GNRMC messages from GPS task via queue and process each one.
        GNRMC incoming;
        if (xQueueReceive(hGNRMC_Queue, &incoming, portMAX_DELAY) == pdTRUE)
        {
            UART_puts("Received new GNRMC data for processing\r\n");
            // process the received message
            gnrmc_localbuffer = incoming;
            parse_GPSdata();

            // drain any additional queued items without blocking
            while (xQueueReceive(hGNRMC_Queue, &incoming, 0) == pdTRUE)
            {
                gnrmc_localbuffer = incoming;
                parse_GPSdata();
            }
        }
        osDelay(1); // Placeholder delay
    }
}