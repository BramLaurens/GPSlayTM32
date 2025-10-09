/*
 * NRF_driver.h
 *
 *  Created on: Sep 23, 2025
 *      Author: braml
 */

#ifndef MYAPP_APP_NRF_DRIVER_H_
#define MYAPP_APP_NRF_DRIVER_H_

#include "GPS_Route_Setter.h"
#include "dGPS.h"

extern void NRF_Driver(void *);
extern uint8_t nrf24_SPI_commscheck(void);
extern void GPS_getlatest_error(dGPS_errorData_t *dest);
#endif