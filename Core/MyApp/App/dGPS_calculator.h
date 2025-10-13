/*
 * dGPS_calculator.h
 *
 *  Created on: Oct 8, 2025
 *      Author: braml
 */

#include "main.h"
#include "cmsis_os.h"
#include "dGPS.h"

#ifndef MYAPP_APP_dGPS_calculator_H_
#define MYAPP_APP_dGPS_calculator_H_

extern void GPS_getlatest_ringbuffer(dGPS_decimalData_t *dest);

#endif /* MYAPP_APP_dGPS_calculator_H_ */