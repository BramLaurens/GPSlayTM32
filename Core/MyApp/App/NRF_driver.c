/*
 * NRF_driver.c
 *
 *  Created on: Sep 23, 2025
 *      Author: braml
 */


#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "NRF_driver.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "NRF24_conf.h"
#include "gps.h"
#include "GPS_Route_Setter.h"

#define PLD_SIZE 32 // Payload size in bytes
uint8_t ack[PLD_SIZE]; // Acknowledgment buffer
uint8_t rx[PLD_SIZE];  // Receive buffer

GPS_decimal_degrees_t errorBuffer;

static GPS_decimal_degrees_t bufferA;
static GPS_decimal_degrees_t bufferB;

static GPS_decimal_degrees_t *volatile frontendBuffer = &bufferA;
static GPS_decimal_degrees_t *volatile backendBuffer  = &bufferB;

extern SPI_HandleTypeDef hspiX;

void GPS_getlatest_error(GPS_decimal_degrees_t *dest)
{
    if(xSemaphoreTake(hdGPSerror_Mutex, portMAX_DELAY) == pdTRUE)
    {
        *dest = *frontendBuffer;
        xSemaphoreGive(hdGPSerror_Mutex);
    }
    else
    {
        error_HaltOS("Err:hdGPSerror_Mutex");
    }
}

void fill_GPSerror()
{
    if(nrf24_data_available())
        {
            #ifdef NRF24_debug
                UART_puts("Data received: \r\n");
            #endif

            nrf24_receive(rx, sizeof(rx)); // Receive data

            //Copy received data to errorBuffer for further processing
            memcpy(&errorBuffer, rx, sizeof(errorBuffer));

            // Debug print
            if(Uart_debug_out & NRF24_DEBUG_OUT)
            {
                char msg[100];
                sprintf(msg, "Lat: %.9f, Lon: %.9f", errorBuffer.latitude, errorBuffer.longitude);
                UART_puts(msg);
                UART_puts("\r\n");
            }
        }

        if(xSemaphoreTake(hdGPSerror_Mutex, portMAX_DELAY) == pdTRUE)
        {
            // Simple buffer swap mechanism
            GPS_decimal_degrees_t *temp = frontendBuffer;
            frontendBuffer = &errorBuffer; 
            backendBuffer = temp;
            xSemaphoreGive(hdGPSerror_Mutex); // Don't forget to give the mutex back
        }
        else
        {
            // Failed to take the mutex (should not happen with portMAX_DELAY)
            error_HaltOS("Err:hdGPSerror_Mutex");
        }
}

void NRF_Driver(void *argument)
{
    osDelay(100);

    UART_puts((char *)__func__); UART_puts(" started\r\n");
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    HAL_GPIO_WritePin(ce_gpio_port, ce_gpio_pin, 0); // Set CE low
    HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin, 1); // Set CSN high


    nrf24_init(); // Initialize NRF24L01+
    nrf24_tx_pwr(_0dbm); // Set TX power to 0dBm
    nrf24_data_rate(_1mbps); // Set data rate to 1Mbps
    nrf24_set_channel(78); // Set channel to 76
    nrf24_pipe_pld_size(0, PLD_SIZE); // Set payload size for pipe 0
    // nrf24_pipe_pld_size(0, sizeof(GPS_decimal_degrees_t)); // Set payload size for pipe 0
    nrf24_set_crc(en_crc, _1byte); // Enable CRC with 1 byte
    nrf24_open_rx_pipe(0, addr); // Open TX pipe with address

    nrf24_pwr_up(); // Power up the NRF24L01+

    nrf24_listen(); // Enter listening mode

    while (TRUE)
    {
        fill_GPSerror();
        osDelay(1); // Placeholder delay
    }
}

uint8_t nrf24_SPI_commscheck(void) {
    uint8_t tx[2] = {0x00, 0xFF};   // R_REGISTER + CONFIG(0x00), then dummy 0xFF
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 50; i++) __NOP();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    for (volatile int i = 0; i < 50; i++) __NOP();
    HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin, GPIO_PIN_SET);

    // rx[0] = STATUS, rx[1] = CONFIG
    return rx[1];
}