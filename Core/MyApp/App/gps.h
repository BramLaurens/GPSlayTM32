/**
* @file my_gps.h
* @brief Bevat basic (provisorische) defines & externals voor de gps routines
* @attention
* <h3>&copy; Copyright (c) 2023 (HU) Michiel Scager.</h3>
* @author MSC
*
* @date 5/9/2023
*/
#ifndef MYAPP_APP_GPS_H_
#define MYAPP_APP_GPS_H_
int hex2int(char *c);
int hexchar2int(char c);
int checksum_valid(char *string);

#include "GPS_Route_Setter.h" // for GPS_decimal_degrees_t
#include "dGPS.h"            // for dGPS_errorData_t, PdGPS_errorData_t

// GNRMC struct: all with char-members - should/could be improved with proper data-elements
typedef struct _GNRMC
{
	char    head[7];       // 0. header
	char    time[10];      // 1. hhmmss.sss
	char    status;        // 2. A=valid, V=not valid
	char    latitude[11];  // 3. ddmm.mmmm (double)
	char    NS_ind;        // 4. N,S
	char    longitude[12]; // 5. ddmm.mmmm (double)
	char    EW_ind;        // 6. E,W
	char    speed[6];      // 7. 0.13 knots (double)
	char    course[6];     // 8. 309.62 degrees (double)
	char    date[7];       // 9. ddmmyy
	char    mag_var[6];    // 10.E,W degrees (double)
	char    mag_var_pos;   // 11.
	char    mode;          // 12.A=autonomous, D,E
	char    cs[4];         // 13.checkum *34
} GNRMC;

typedef struct _GNGGA
{
	// To be defined if needed
	char   head[7];        // 0. header
	char   time[10];       // 1. hhmmss.sss
	char   latitude[11];   // 2. ddmm.mmmm (double)
	char   NS_ind;         // 3. N,S
	char   longitude[12];  // 4. ddmm.mmmm (double)
	char   EW_ind;         // 5. E,W
	char   fix_quality;    // 6. 0 = invalid, 1 = GPS fix, 2 = DGPS fix
	char   num_satellites[3]; // 7. number of satellites being tracked
	char   horizontal_dilution[5]; // 8. horizontal dilution of position (double)
	char   altitude[8];    // 9. altitude (double)
	char   ellipsoid_height[8]; // 10. height of geoid above WGS84 ellipsoid (double)
	char   time_since_last_DGPS[6]; // 11. time in seconds since last
	char   DGPS_station_ID[4]; // 12. DGPS station ID number
	char   cs[4];          // 13. checksum *hh
} GNGGA;


// enum voor NMEA protocolstrings (starting 'e' for enum)
enum NMEA
{
	eGNRMC = 1,
	eGPGSA,
	eGNGGA
};

// Expose function to get pointer to latest complete GNRMC data

void getlatest_GNRMC(GNRMC *dest);
void correct_dGPS_error(PdGPS_errorData_t pinputCoordinates);
void GPS_get_fix_quality(char *dest);

#endif /* MYAPP_APP_GPS_H_ */
