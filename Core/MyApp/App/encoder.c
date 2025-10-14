/**
@brief Dit bestand bevat de functies voor de encoder
 */
#include "main.h"
#include "cmsis_os.h"
#include "encoder.h"
#include "admin.h"

double counterLinks = 0;
double counterRechts = 0;


void Enc_task(void) //uitproberen in de main?
{

    while (TRUE)
    {
    	CheckLinksRechts();
    }
}

void CheckLinksRechts()
{
	uint8_t lastStateLinks1 = 0;
	uint8_t lastStateRechts2 = 0;
	uint8_t currentStateLinks1;
	uint8_t currentStateRechts2;
	   // start timer ergens?
	    currentStateLinks1 = HAL_GPIO_ReadPin(GPIOC, ENCODER_A_2_Pin);
	    currentStateRechts2 = HAL_GPIO_ReadPin(GPIOC, ENCODER_B_2_Pin);

	    if (currentStateLinks1 == GPIO_PIN_SET && lastStateLinks1 == GPIO_PIN_RESET) // Signal High <-> Low received
	    {
	    counterLinks++;
	    }
	    if (currentStateRechts2 == GPIO_PIN_SET && lastStateRechts2 == GPIO_PIN_RESET)
	    {
	    counterRechts++;
	    }

	    if (counterLinks - counterRechts >=4)
	    {
	        UART_puts("Links gaat beetje harder\n\r");
	    }
		else if (counterLinks - counterRechts >= 8)
		{
			UART_puts("Links gaat veel harder\n\r");
		}
	    else if (counterRechts - counterLinks >= 4)
	    {
	        UART_puts("Rechts gaat beetje harder\n\r");
	    }
		else if (counterRechts - counterLinks >= 8)
		{
			UART_puts("Rechts gaat veel harder\n\r");
		}
	    else
	    {
	        UART_puts("Rijd gelijk\n\r");
	    }
	    osDelay(100);

	    lastStateLinks1 = currentStateLinks1;
	    lastStateRechts2 = currentStateRechts2;
}

void TimerEnc_Handler(void) //Timer die de counters reset
{
    counterLinks= 0; //
    counterRechts = 0;

}
