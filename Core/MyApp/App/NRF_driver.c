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
#include <stdio.h>

#define PLD_SIZE 32 // Payload size in bytes
uint8_t rx[PLD_SIZE]; // Transmission buffer
uint8_t ack[PLD_SIZE]; // Acknowledgment buffer
GPS_decimal_degrees_t latestError;

extern SPI_HandleTypeDef hspiX;

/**
 * @brief TEST!! receiver function for the NRF24 module in receiver mode. Checks continously for new data and stores in RX buffer.
 * 
 * @param argument 
 */
void NRF_testreceive()
{
    nrf24_listen(); // Enter listening mode

    if(nrf24_data_available())
    {
        UART_puts("NRF24 Data available!\r\n");

        nrf24_receive(rx, PLD_SIZE);
        char msg[100];
        /* Print the received raw data */
        sprintf(msg, sizeof(msg));
        UART_puts(msg);
    }
}


/**
 * @brief Receiver function for the NRF24 module in receiver mode. Checks continously for new data and stores in RX buffer.
 * 
 * @param argument 
 */
void NRF_receive()
{
    nrf24_listen(); // Enter listening mode

    if(nrf24_data_available())
    {
        UART_puts("NRF24 Data available!\r\n");

        nrf24_receive(rx, PLD_SIZE);
        char msg[100];
        /* Print the received raw data as hex values. */
        snprintf(msg, sizeof(msg), "NRF RX: ");
        UART_puts(msg);
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

/**
 * @brief Driver for the NRF24 module in receiver mode. Checks continously for new data.
 * 
 * @param argument 
 */
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
    nrf24_set_crc(en_crc, _1byte); // Enable CRC with 1 byte
    nrf24_open_rx_pipe(0, addr); // Open TX pipe with address

    nrf24_pwr_up(); // Power up the NRF24L01+
    
    while (TRUE)
    {
        NRF_testreceive();
        osDelay(1); // Placeholder delay
    }
}
