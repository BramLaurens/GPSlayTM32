/**
* @file gps.c
* @brief Behandelt de gps input-strings (NMEA-protocol) van UART1.<br>
* <b>Demonstreert: xMessageBufferRead() </b><br>
* Aan UART1 is een interrupt gekoppeld (zie main.c: HAL_UART_RxCpltCallback(),
* die de inkomende string op een messagebuffer zet, die we hier uitlezen en verwerken.<br>
* @author MSC
*
* @date 5/5/2023
*/
#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "gps.h"


GNRMC gnrmc; // global struct for GNRMC-messages

static GNRMC bufferA;
static GNRMC bufferB;

static GNRMC *volatile frontendBuffer = &bufferA; 
static GNRMC *volatile backendBuffer  = &bufferB; 

/**
 * @brief Corrects the input coordinates with the latest GPS error received from the NRF24L01+ module
 * 
 * @param pinputCoordinates pointer to GPS_decimal_degrees_t struct containing the coordinates to be corrected
 * @return void
 */
void correct_dGPS_error(PdGPS_errorData_t pinputCoordinates)
{
	// Get the latest error from the NRF24L01+ module
	dGPS_errorData_t latestError;
	GPS_getlatest_error(&latestError);

	#ifdef dGPS_debug
		UART_puts("Correcting error\r\n");
		char msg[100];
		sprintf(msg, "Working Error - Lat: %.9f, Lon: %.9f\r\n", latestError.latitude, latestError.longitude);
		UART_puts(msg);

		sprintf(msg, "Before correction - Lat: %.9f, Lon: %.9f\r\n", pinputCoordinates->latitude, pinputCoordinates->longitude);
		UART_puts(msg);
	#endif

	// Apply the correction
	pinputCoordinates->latitude -= latestError.latitude;
	pinputCoordinates->longitude -= latestError.longitude;

	#ifdef dGPS_debug
		sprintf(msg, "After correction - Lat: %.6f, Lon: %.6f\r\n", pinputCoordinates->latitude, pinputCoordinates->longitude);
		UART_puts(msg);
	#endif

	return;
}

/**
 * @brief Function that gets a pointer to the latest complete GNRMC data
 * 
 * @param dest pointer of type GNRMC that will be pointing to the latest GNRMC data
 * @return void
 */
void getlatest_GNRMC(GNRMC *dest)
{
	if(xSemaphoreTake(hGPS_Mutex, portMAX_DELAY) == pdTRUE)
	{
		*dest = *frontendBuffer;
		xSemaphoreGive(hGPS_Mutex);
	}
	else
	{
		error_HaltOS("Err:GPS_mutex");
	}
}

/**
 * @brief Checks the GPS fix status and updates the green LED accordingly.
 * If the GPS status is 'A' (valid), the green LED is turned on, otherwise it is turned off.
 */
void check_gpsfix(GNRMC *gnrmc)
{
	if(gnrmc->status == 'A') // If status is 'A' (valid)
	{
		HAL_GPIO_WritePin(GPIOD, LEDGREEN, GPIO_PIN_SET); // Turn on green LED for GPS LOCK
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, LEDGREEN, GPIO_PIN_RESET); // Turn off green LED if no GPS lock
	}
}

void fill_GNGGA(char *message)
{
	// To be implemented if needed

	char *tok = ",";
	char *s;

	GNGGA localBuffer;
	memset(&localBuffer, 0, sizeof(GNGGA)); // clear the struct

	s = strsep(&message, tok); // 0. header;
	strcpy(localBuffer.head, s);

	s = strsep(&message, tok);    // 1. time; not used
	strcpy(localBuffer.time, s);

	s = strsep(&message, tok);    // 2. latitude;
	strcpy(localBuffer.latitude, s);

	s = strsep(&message, tok);    // 3. N/S; not used

	s = strsep(&message, tok);    // 4. longitude;
	/* Remove at most two leading zeros if present, but avoid turning "0.xxx" into ".xxx".
	   Only strip a second leading '0' when the following character is a digit (not '.')
	   and when there are at least two characters. */
	if (s[0] == '0') {
		size_t len = strlen(s);
		if (len > 1 && s[1] == '0') {
			/* s starts with "00" -> remove first zero */
			memmove(s, s + 1, len);
			/* Now s may still start with '0'. If the next char after the remaining leading '0' is a digit
			   (not a dot) we can remove it too, otherwise leave the single leading zero to preserve values like "0.123" */
			len = strlen(s);
			if (len > 1 && s[0] == '0' && s[1] != '.') {
				memmove(s, s + 1, len);
			}
		} else {
			/* single leading zero: only remove it if the next char isn't '.' to avoid ".xxx" */
			if (len > 1 && s[1] != '.') {
				memmove(s, s + 1, strlen(s));
			}
		}
	}
	strcpy(localBuffer.longitude, s);

	s = strsep(&message, tok);    // 5. E/W; not used

	s = strsep(&message, tok);    // 6. fix quality;
	localBuffer.fix_quality = s[0];

	s = strsep(&message, tok);    // 7. number of satellites;
	strcpy(localBuffer.num_satellites, s);

	s = strsep(&message, tok);    // 8. horizontal dilution;
	strcpy(localBuffer.horizontal_dilution, s);

	s = strsep(&message, tok);    // 9. altitude;
	strcpy(localBuffer.altitude, s);

	s = strsep(&message, tok);    // 10. height of geoid above WGS84 ellipsoid;
	strcpy(localBuffer.ellipsoid_height, s);

	s = strsep(&message, tok);    // 11. time in seconds since last DGPS update;
	strcpy(localBuffer.time_since_last_DGPS, s);

	s = strsep(&message, tok);    // 12. DGPS station ID number;
	strcpy(localBuffer.DGPS_station_ID, s);

	if(localBuffer.fix_quality == '4' || localBuffer.fix_quality == '5'){
		HAL_GPIO_WritePin(GPIOD, LEDBLUE, GPIO_PIN_SET); // RTK fix
		LCD_clear();
		LCD_puts("RTK fix! Wohoo");
	}
	else{
		HAL_GPIO_WritePin(GPIOD, LEDBLUE, GPIO_PIN_RESET); // no RTK fix
		LCD_clear();
		LCD_puts("No RTK fix");
	}

}

/**
* @brief De chars van de binnengekomen GNRMC-string worden in data omgezet, dwz in een
* GNRMC-struct, mbv strsep(); De struct bevat nu alleen chars - je kunt er ook voor kiezen
* om gelijk met doubles te werken, die je dan met atof(); omzet.
* @return void
*/
void fill_GNRMC(char *message)
{
	// example: $GNRMC,164435.000,A,5205.9505,N,00507.0873,E,0.49,21.70,140423,,,A
	//          id    , time     ,s,

	osThreadId_t hTask;

	char *tok = ",";
	char *s;

	GNRMC *localBuffer = backendBuffer;

	memset(localBuffer, 0, sizeof(GNRMC)); // clear the struct

	s = strsep(&message, tok); // 0. header;
	strcpy(localBuffer->head, s);

	s = strsep(&message, tok);    // 1. time; not used
	strcpy(localBuffer->time, s);

	s = strsep(&message, tok);    // 2. valid;
	localBuffer->status = s[0];

	s = strsep(&message, tok);    // 3. latitude;
	strcpy(localBuffer->latitude, s);

	s = strsep(&message, tok);    // 4. N/S; not used

	s = strsep(&message, tok);    // 5. longitude;
	/* Same robust handling for GNRMC longitude as above */
	if (s[0] == '0') {
		size_t len = strlen(s);
		if (len > 1 && s[1] == '0') {
			memmove(s, s + 1, len);
			len = strlen(s);
			if (len > 1 && s[0] == '0' && s[1] != '.') {
				memmove(s, s + 1, len);
			}
		} else {
			if (len > 1 && s[1] != '.') {
				memmove(s, s + 1, strlen(s));
			}
		}
	}
	strcpy(localBuffer->longitude, s);

	s = strsep(&message, tok);    // 6. E/W; not used

	s = strsep(&message, tok);    // 7. speed;
	strcpy(localBuffer->speed, s);

	s = strsep(&message, tok);    // 8. course;
	strcpy(localBuffer->course, s);

	if (Uart_debug_out & GPS_DEBUG_OUT)
	{
		UART_puts("\r\n\t GPS type: \t");  UART_puts(localBuffer->head);
		UART_puts("\r\n\t status:   \t");  UART_putchar(localBuffer->status);
		UART_puts("\r\n\t latitude: \t"); UART_puts(localBuffer->latitude);
		UART_puts("\r\n\t longitude:\t");  UART_puts(localBuffer->longitude);
		UART_puts("\r\n\t speed:    \t");  UART_puts(localBuffer->speed);
		UART_puts("\r\n\t course:   \t");  UART_puts(localBuffer->course);
	}

	if(localBuffer->status == 'A'){
		HAL_GPIO_WritePin(GPIOD, LEDGREEN, GPIO_PIN_SET); // GPS fix
	}
	else{
		HAL_GPIO_WritePin(GPIOD, LEDGREEN, GPIO_PIN_RESET); // no GPS fix
	}

	if(xSemaphoreTake(hGPS_Mutex, portMAX_DELAY) == pdTRUE)
	{
		GNRMC *tempbuf = backendBuffer;
		backendBuffer = frontendBuffer;
		frontendBuffer = tempbuf;
		xSemaphoreGive(hGPS_Mutex);
	}
	else
	{
		error_HaltOS("Err:GPS_mutex");
	}

	check_gpsfix(frontendBuffer);

	// Send a copy of the latest GNRMC data to the dGPS task via a queue so the
	// dGPS task can process every incoming data point
	if (xQueueSend(hGNRMC_Queue, frontendBuffer, 0) != pdPASS)
	{
		// Queue full: drop oldest item then enqueue
		GNRMC tmp;
		xQueueReceive(hGNRMC_Queue, &tmp, 0);
		xQueueSend(hGNRMC_Queue, frontendBuffer, 0);
	}
}


/**
* @brief Leest de GPS-NMEA-strings die via de UART via interrupt-handler (HAL_UART_RxCpltCallback)
* binnenkomen. * De handler zet elk inkomende character gelijk op een queue, die hier uitgelezen wordt.
* Vervolgens wordt hiervan een GPS-message opgebouwd en verwerkt.
* @return void
*/
void GPS_getNMEA (void *argument)
{
    char  Q_char;   			// char to receive from queue
	char  MSG_buff[GPS_MAXLEN]; // buffer for GPS-string
	int   pos = 0;
	int   cs;                   // checksum-flag
	int   new_msg = FALSE;      // do we encounter a '$'-char?
	int   msg_type = 0;         // do we want this message to be interpreted?

	UART_puts((char *)__func__); UART_puts("started\n\r");

	while (TRUE)
	{
		xQueueReceive(hGPS_Queue, &Q_char, portMAX_DELAY); // get one char from the q

		//UART_putchar(Q_buff);  // echo, for testing

		if (Q_char == '$') // gotcha, new datastring started
		{
			memset(MSG_buff, 0, sizeof(MSG_buff)); // clear buff
			pos = 0;
			new_msg = TRUE; // from now on, chars are valid to receive
		}

		if (new_msg == FALSE) // char only valid if started by $
			continue;

		MSG_buff[pos] = Q_char; // copy char read from Q into the msg-buf

		// if pos==5, the message type (f.i. "$GPGSA) is complete, so we now we can determine
		// if we want the rest of the message... else we skip the rest characters
		if (pos == 5)
		{
			msg_type = 0; // reset

			// next, we decide which message types we want to interpret
			// and we set the message-type for later use...
			// Accept sentences regardless of talker ID (e.g. GPGGA, GNGGA, etc.)
			// Check the 3-letter sentence type (chars 3..5 of the header, indexes 3-5 in MSG_buff starting at 1)
			// MSG_buff layout when pos==5: [0]='$', [1]='G', [2]='N'|'P'.., [3]='G', [4]='G', [5]='A'
			if (MSG_buff[3] == 'R' && MSG_buff[4] == 'M' && MSG_buff[5] == 'C') msg_type = eGNRMC; // *RMC
			else if (MSG_buff[3] == 'G' && MSG_buff[4] == 'G' && MSG_buff[5] == 'A') msg_type = eGNGGA; // *GGA

			if (!msg_type) // not an interesting message type
			{
				new_msg = FALSE;
				continue;
			}
		}

		// if we are here, we are reading the rest of the message into the msg_buff
		////////////////////////////////////////////////////////////////////////////
		if (pos >= GPS_MAXLEN - 1) // avoid overflow (should not happen, but still...)
		{
			new_msg = FALSE; // ignore it
			continue;
		}

		if (MSG_buff[pos] == '\r') // end of message encountered - all messages end with <CR-13><LF-10>
		{
			MSG_buff[pos] = '\0';          // close string
			cs = checksum_valid(MSG_buff); // note, checksumchars (eg "*43") are removed from string

			if (Uart_debug_out & GPS_DEBUG_OUT) // output to uart if wanted
			{
				UART_puts("\r\nGPS (UART4): "); UART_puts(MSG_buff);
				UART_puts( cs ? " [cs:OK]\r\n" : " [cs:ERR]\r\n");
			}

			if (cs) // checksum okay, so interpret the message
			{
				switch(msg_type) // extract data from msg into right struct
				{
				case eGNRMC: 
							fill_GNRMC(MSG_buff);
						    break;
				case eGPGSA:
				case eGNGGA: 
							fill_GNGGA(MSG_buff);
							break;
				default:    break;
				}
			}
			else
			{
				if (Uart_debug_out & GPS_DEBUG_OUT)
					UART_puts("GPS message dropped due to invalid checksum\r\n");
			}

			new_msg = FALSE; // new message possible
			continue;
		}
		pos++; // proceed reading next char from the queue
	}
}


// source: file:///C:/craigpeacock/NMEA-GPS
int hex2int(char *c)
{
	int value;

	value = hexchar2int(c[0]);
	value = value << 4;
	value += hexchar2int(c[1]);

	return value;
}


int hexchar2int(char c)
{
    if (c >= '0' && c <= '9')
        return (c - '0');
    if (c >= 'A' && c <= 'F')
        return (c - 'A' + 10);
    if (c >= 'a' && c <= 'f')
        return (c - 'a' + 10);
    return (-1);
}


// source: file:///C:/craigpeacock/NMEA-GPS
int checksum_valid(char *string)
{
	char *checksum_str;
	int checksum, i;
	unsigned char calculated_checksum = 0;

	// Checksum is postcede by *
	if ((checksum_str = strchr(string, '*')))
	{
		*checksum_str = '\0'; // Remove checksum from string
		// Calculate checksum, starting after $ (i = 1)
		for (i = 1; i < strlen(string); i++)
			calculated_checksum = calculated_checksum ^ string[i];

		checksum = hex2int((char *)checksum_str+1);
		//printf("Checksum Str [%s], Checksum %02X, Calculated Checksum %02X\r\n",(char *)checksum_str+1, checksum, calculated_checksum);
		if (checksum == calculated_checksum)
			return (1);
	}

	return (0);
}
