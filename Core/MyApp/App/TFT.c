#include "main.h"
#include "cmsis_os.h"
#include "admin.h"
#include "st7735.h"


#include <stdint.h>

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
    ST7735_WriteString(10, 10, "Hello, TFT!", Font_11x18, ST7735_WHITE, ST7735_BLACK);
    
    UART_puts("TFT display initialized.\r\n");


    while (TRUE)
    {
        // Update display content here as needed
        osDelay(1000);
    }
}