/*
Author: 	J.F. van der Bent
			W Pielage & E Helmond

Date:		24-11-2021
Revision:	5

    uart.c:
          UART2 driver for ARM-board V5

    pin-info:
           PA2 - TX
           PA3 - RX

To enable UART use the following line:
	UART_init();
To use UART on interrupt base use:
	UART_INT_init();

This file initialize the UART on the ARM-board v5.
To send data to the UART use:
	UART_printf(*string);


In the interrupt routine the char is send back to the terminal
07-07-2014 Aanpassen Uart routine om compatibel te zijn met void UART_put(char *c)
24011-2021 aanpassen van de Uart lib naar HAL

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "string.h"

#include "uart.h"


char string[100];
int charcounter = 0;
extern UART_HandleTypeDef huart2;

void UART_init(void)
{

 // done in CUBE

}

void UART_putchar(unsigned char c)
{
	 HAL_UART_Transmit(&huart2, &c, 1, 100);
}

void UART_puts(const char *s)
{
	volatile unsigned int i;

	for (i=0; s[i]; i++)
		UART_putchar(s[i]);
}


void UART_INT_init(void)
{
	//not used
}

signed int UART_printf(size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;
    char c[length];
    char *pString = c;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);

    __asm( "CPSID   I");
    volatile unsigned char i;
	for (i=0; pString[i]; i++)
	{
		 UART_putchar(pString[i]);
	}
	__asm( "CPSIE   I");

    return rc;
}

// Stuurt meegegeven getal uit op de UART
void UART_putint(unsigned int num)
{
    UART_putnum(num, 10);
}

// Stuurt meegegeven getal uit op de UART in het aangegeven getallenstelsel
void UART_putnum(unsigned int num, unsigned char deel)
{
    static unsigned char chars[16] = "0123456789ABCDEF";
    unsigned int rest;
    signed char c[16];
    signed int i=15;

    // Zet de integer om naar een string
    if(num==0)
    {
        c[i]='0';
        i--;
    }
    else
    {
        while(num>0)
        {
            rest=num%deel;
            num/=deel;
            c[i]=chars[rest];
            i--;

            if(i==0) // it ends here
                num=0;
        }
    }

    // Stuur de string uit
    while(i<15)
    {
        i++;
        UART_putchar(c[i]);
    }
}


// Ontvang een karakter via de UART2 middels pollen. Wanneer de NVIC aan wordt gezet (in de main wordt hij uitgezet)
// niet echt nodig als routine maar als wrapper voor compatabiliteit. Let op geen -1 als er geen char is ontvangen!

char UART_get(void)
{
    unsigned char uart_char = -1;

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != 0)		// check for data available using HAL
    {
    	uart_char = USART2->DR & 0xFF;							// and read the data from peripheral
    	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);			// clear data available flag using HAL
    }
    return uart_char;
}


// void UART_gets
// args: char *readbuffer
//		 int size, size of readbuffer handed over
//       int   echo, when TRUE, send read-char to UART
// remark: ARM sends -1 if buffer is empty
//         LF is cleared if set in terminal-program
void UART_gets(char *s, int size, int echo)
{
	int pos = 0;

	while (1)
	{
		// ADD DELAY FOR SCHEDULER, IF TOO LONG, CHARS CAN BE MISSED...
		osDelay(60);

		*s = UART_get();

		if (*s == -1 || *s == 255 || *s == CRETURN) // no data, or CR pressed
			continue;

		UART_putchar(*s);  // show character

		if (*s == LFEED)          // check LF pressed
		{
			*s = '\0';            // now, close string
			return;
		}
		else if (pos == size - 2) // check if last char reached
		{
			*++s = '\0';          // first, skip to last position
			return;
		}

		s++;
		pos++;
	}
}
