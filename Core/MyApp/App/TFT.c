#include "main.h"
#include "cmsis_os.h"
#include "admin.h"
#include "st7735.h"
#include "gps.h"
#include "routeperformer.h"
#include "GPS_Route_Setter.h"
#include "LOS_algo.h"

#include <stdint.h>

char GPS_fix_quality_local = 0;
char GPS_fix_quality_str[20];

void update_GPS_fix_quality_display()
{
    GPS_get_fix_quality(&GPS_fix_quality_local);
    switch (GPS_fix_quality_local)
    {
        case 0:
            sprintf(GPS_fix_quality_str, "No Fix");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_RED);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_RED);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_RED);
            break;
        case 1:
            sprintf(GPS_fix_quality_str, "GPS Fix");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_BLUE);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_BLUE);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_BLUE);
            break;
        case 2:
            sprintf(GPS_fix_quality_str, "DGPS Fix");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_BLUE);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_BLUE);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_BLUE);
            break;
        case 4:
            sprintf(GPS_fix_quality_str, "RTK Fix");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_GREEN);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_GREEN);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_GREEN);
            break;
        case 5:
            sprintf(GPS_fix_quality_str, "RTK Float");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_GREEN);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_GREEN);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_GREEN);
            break;
        default:
            sprintf(GPS_fix_quality_str, "Unknown");
            ST7735_FillRectangleFast(0, 30, 160, 10, ST7735_RED);
            ST7735_WriteString(10, 30, "GPS Status: ", Font_7x10, ST7735_WHITE, ST7735_RED);
            ST7735_WriteString(90, 30, GPS_fix_quality_str, Font_7x10, ST7735_WHITE, ST7735_RED);
            break;
    }
}

void updateWPdisplay()
{
    bool RP_algostate;

    ST7735_WriteString(10, 40, "WP Amount: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    char wp_amount_str[5];
    char wp_amount = 0;
    RS_getWPamount(&wp_amount);
    sprintf(wp_amount_str, "%d", wp_amount);
    ST7735_WriteString(100, 40, wp_amount_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    get_RP_AlgoState(&RP_algostate);

    if(RP_algostate)
    {
        ST7735_FillRectangleFast(10, 50, 160, 10, ST7735_BLACK);
        ST7735_WriteString(10, 50, "Going to: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
        char wp_current_str[5];
        int wp_current = 0;
        // RP_get_wpCurrent(&wp_current);
        LOS_getcurrentWPnumber(&wp_current);
        sprintf(wp_current_str, "%d", wp_current+1);
        ST7735_WriteString(100, 50, wp_current_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
    }
    else
    {   
        ST7735_FillRectangleFast(10, 50, 160, 10, ST7735_BLUE);
        ST7735_WriteString(10, 50, "Going to: ", Font_7x10, ST7735_WHITE, ST7735_BLUE);
        ST7735_WriteString(100, 50, "PAUSED", Font_7x10, ST7735_WHITE, ST7735_BLUE);
    }
}

void update_routedata_display()
{
    // This function can be used to update additional route data on the display if needed
    ST7735_WriteString(10, 60, "Dist WP:", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    char distance_str[10];
    double distance = 0.0;
    // get_RP_distance(&distance);
    LOS_getWPdistance(&distance);
    sprintf(distance_str, "%2.2f m", distance);
    ST7735_WriteString(100, 60, distance_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    ST7735_WriteString(10, 70, "Course WP:", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    char course_str[10];
    double course = 0.0;
    // getlatestCourse(&course);
    LOS_getWPbearing(&course);
    sprintf(course_str, "%2.2f", course);
    ST7735_WriteString(100, 70, course_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void TFT_task(void *argument)
{
    UART_puts((char *)__func__); UART_puts(" started\r\n");
    osDelay(1000); // Wait for system to stabilize
    UART_puts("Initializing TFT display...\r\n");

    // Initialize the display and run visible-only test pattern (no MISO needed)
    ST7735_Unselect();
    ST7735_Init();
    ST7735_FillScreen(ST7735_BLACK);

    // Visible test pattern for write-only displays
    // ST7735_TestPattern();

    // Then write a final message
    ST7735_WriteString(10, 10, "GPSLAY Rover", Font_11x18, ST7735_WHITE, ST7735_BLACK);
    ST7735_WriteString(10, 30, "Not a bomb!", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    
    UART_puts("TFT display initialized.\r\n");


    while (TRUE)
    {
        // Update display content here as needed
        osDelay(1000);
        ST7735_WriteString(10, 10, "GPSLAY Rover", Font_11x18, ST7735_WHITE, ST7735_BLACK);
        update_GPS_fix_quality_display();
        updateWPdisplay();
        update_routedata_display();
    }
}
