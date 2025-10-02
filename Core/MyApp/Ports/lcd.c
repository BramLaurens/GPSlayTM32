/*
Author: 	J.F. van der Bent
			W Pielage & E Helmond

Date:		24-11-2021
Revision:	5
    lcd.c:
          LCD-driver ARM-board v5

    pin-info:
           PE2  - DB4   --->> PE3
           PE4  - DB5
           PE5  - DB6
           PE6  - DB7
           PC13 - E:  start data
           PC15 - RS: register-select
           PC14 - RW: read/write

    opmerkingen:
                -Alle instructies voor het display staan achterstevoren! (maakt
                 de code wat eenvoudiger)
                -Zorg ervoor dat de delay-code ook wordt meegecompileerd, anders
                 gaat het hier mis...
                -Deze code is gebaseerd op
                 "LCD routines for OLIMEX LPC-MT-2106,16x2 Character LCD 1602K,
                  4-bit mode operation",
                 Copyright (c) 2004 senz at arm.dreamislife.com
                 15-07-2014 aanpassing voor 1x16 display define eenregel bepaald type display J.F. van der Bent
                 20-10-2021 aanpassing naar de HAL lib
*/

#include "main.h"
#include "lcd.h"

static void LCD_writenibble(unsigned char data);
static void LCD_writebyte(unsigned char data);

unsigned char curpos = 0; // remember cursorposition

void ClearBits(void)
{
	HAL_GPIO_WritePin(LCD_D4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7, GPIO_PIN_RESET);
}

void LCD_cursor_home(void)
{
   LCD_writecontrol(0x02); // cursur home
   curpos=0;               // reset position
}

void LCD_clear(void)
{
   LCD_writecontrol(0x01); // clearscreen
   curpos=0;               // reset position
   HAL_Delay(5);
//   busyflag();

}

void LCD_XY(unsigned int x, unsigned int y)
{
	unsigned char data;

	switch(y)
	{
	case 0:  data = (1<<7)+0x00+x; break;
	case 1:  data = (1<<7)+0x40+x; break;
	case 2:  data = (1<<7)+0x14+x; break;
	default: data = (1<<7)+0x54+x; break;
	}
	LCD_writecontrol(data);
}


// init LCD
// Display wordt gebruikt in 4bits modus,2 regels, 5x7 dots font.
void LCD_init(void)
{
	GPIO_InitTypeDef gpio;	// GPIO init structure
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	HAL_GPIO_Init(GPIOE, &gpio);

	gpio.Pin = GPIO_PIN_11 | GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &gpio);

	gpio.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOD, &gpio);

//	GPIOE -> MODER |= ( 1 << 4 );
//	GPIOE -> MODER |= ( 1 << 8 );
//	GPIOE -> MODER |= ( 1 << 10 );
//	GPIOE -> MODER |= ( 1 << 12 );
//	GPIOC -> MODER |= ( 1 << 26 );
//	GPIOC -> MODER |= ( 1 << 30 );
//	GPIOC -> MODER |= ( 1 << 28 );

	HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RW, GPIO_PIN_RESET);

	HAL_Delay(15);//15

    LCD_writebyte(0x30); // function-set boot
    HAL_Delay(70);
    LCD_writebyte(0x30);
    LCD_writebyte(0x30);

     // interface op 4 bits
    LCD_writebyte(0x02);
    HAL_Delay(5);

    LCD_writebyte(0x2C);  // function set
    HAL_Delay(15);
    LCD_writebyte(0x0C);  // lcd aan
    HAL_Delay(15);
    LCD_writebyte(0x01);  // clear screen
    HAL_Delay(15);
    LCD_writebyte(0x06);  // entry mode set
    HAL_Delay(15);
}

// Zet meegegeven karakter op het scherm
void LCD_putchar(char c)
{
    HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_SET);
    LCD_writebyte(c);
    if (++curpos==32) // remember cursorpos
    	curpos=0;
}

// Zet meegegeven string op het scherm
void LCD_put(char *string)
{
#ifdef eenregel
   	unsigned char j;
#endif
    unsigned char k;
	// vaag display; teken 8 tot 31 is niet uitgevoerd op het scherm
    for (k=0; string[k]; k++)
	{

#ifdef eenregel
    	// vaag display; teken 8 tot 31 is niet uitgevoerd op het scherm
    	if(curpos==16) // at 8, goto next valid lcd-position
        for (j=0; j<23; j++) LCD_cursor_right(); // 23 vult het hele scherm met 32 chars idk why xxx Jasper
#endif
        LCD_putchar(string[k]);
    }
}

void LCD_puts(char *c)
{
    LCD_put(c);
}

// Zet het meegegeven positief getal op het scherm
void LCD_putint(unsigned int num)
{
    unsigned int rest;
    static unsigned char chars[10] = "0123456789"; // possible chars
    char s[17]; // display has 16 chars [0..15] + 1 closing null
	s[16]='\0'; // closing null set
    int i=15;   // startposition at last display char

    // kinda function itoa, integer to ascii
    if(num==0)
        s[i--]='0'; // '0' at 14
    else
	{
        while(num>0 && i)
		{
            rest = num%10;
            num /= 10;
            s[i--]=chars[rest];
        }
    }
	LCD_put(&s[i+1]); // first, set i to starting pos, undo one i-- too much
}

// Stuurt een 4-bits commando naar het display
static void LCD_writenibble(unsigned char data)
{
    ClearBits();

    if (data & 0x01)
		HAL_GPIO_WritePin(LCD_D7, GPIO_PIN_SET);

	if (data & 0x02)
		HAL_GPIO_WritePin(LCD_D6, GPIO_PIN_SET);

	if (data & 0x04)
		HAL_GPIO_WritePin(LCD_D5, GPIO_PIN_SET);

	if (data & 0x08)
		HAL_GPIO_WritePin(LCD_D4, GPIO_PIN_SET);

}


// Stuurt een 8-bits commando naar het display
static void LCD_writebyte(unsigned char data)
{
    /* hoogste 4 bits */
    HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_SET);
    LCD_writenibble((data>>4)&0x0F);
    HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_RESET);

    HAL_Delay(2);

    /* laagste 4 bits */
    HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_SET);
    LCD_writenibble(data&0x0F);
    HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_RESET);

    HAL_Delay(2);
}

// Stuurt een commando naar het display
void LCD_writecontrol(unsigned char data)
{
    HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_RESET);
    LCD_writebyte(data);
}


void busyflag(void)
{
//	uint8_t bitstatus = 0x00;
//	uint8_t bitstatus1 = GPIO_PIN_SET;
//	GPIO_InitTypeDef gpio;
//
//	while(bitstatus == bitstatus1)
//	{
//		gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
//		gpio.Mode = GPIO_MODE_INPUT;
//		gpio.Pull = GPIO_NOPULL;
//		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		HAL_GPIO_Init(GPIOE, &gpio);
//
//		HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(LCD_RW, GPIO_PIN_SET);
//
//		DELAY_ms(1);
//		HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_SET);
//		DELAY_ms(1);
//
//		bitstatus = HAL_GPIO_ReadPin(LCD_D7);
//		HAL_GPIO_WritePin(LCD_EN, GPIO_PIN_RESET);
//		DELAY_ms(1);
//	}
//
//	gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
//	gpio.Mode = GPIO_MODE_OUTPUT_PP;
//	gpio.Pull = GPIO_NOPULL;
//	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOE, &gpio);

	HAL_Delay(1);
}

void LCD_createChar(uint8_t location, uint8_t map[8])
{
	location &= 0x7;
	LCD_writecontrol(LCD_SETCGRAMADDR | (location<< 3));
	for(uint8_t i = 0; i< 8;i++)
	{
		HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_SET);
		LCD_writebyte(map[i]);
	}
}
