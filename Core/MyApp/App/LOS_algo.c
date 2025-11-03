#include "main.h"
#include "cmsis_os.h"
#include "admin.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "GPS_Route_Setter.h"
#include "LOS_algo.h"
#include "gps.h"
#include "compass_driver.h"

#define M_PI 3.14159265358979323846

// --- Constants / tunable parameters ---
#define ARRIVAL_RADIUS_M     1.0    // meters to consider a waypoint reached
#define LOOKAHEAD_DISTANCE_M 2.0    // meters ahead on path segment

// --- Helper: degrees to radians ---
static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

static GPS_Route *prev_wp = NULL;
static GPS_Route *curr_wp = NULL;
LOS_Output_t los;

static double ref_lat, ref_lon;

bool enable_LOS_algo = false; // Flag to enable/disable LOS algorithm
bool hold_LOS_at_waypoint = false; // Flag to hold at waypoint

GNRMC gnrmc_los; // Local copy of GPS data for LOS algorithm

double los_desiredheading = 0.0;

int current_LOS_waypoint_number = -1;

void LOS_getWPbearing(double *dest)
{
    *dest = los.desired_bearing;
}

void LOS_getWPdistance(double *dest)
{
    *dest = los.distance_to_wp;
}

void LOS_getcurrentWPnumber(int *dest)
{
    *dest = current_LOS_waypoint_number;
}

void LOS_getwaypointhold_status(bool *dest)
{
    *dest = hold_LOS_at_waypoint;
}

void LOS_getdesiredheading(double *dest)
{
    *dest = los_desiredheading;
}

void set_LOS_algoState(bool state)
{
    enable_LOS_algo = state;
}

static void latlon_to_xy(double lat_ref, double lon_ref,
                         double lat, double lon,
                         double *x, double *y)
{
    double R = 6371000.0; // Earth radius in meters
    double dLat = deg2rad(lat - lat_ref);
    double dLon = deg2rad(lon - lon_ref);
    double lat_ref_rad = deg2rad(lat_ref);
    *x = R * dLon * cos(lat_ref_rad);
    *y = R * dLat;
}

static double bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
    double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) -
               sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
    double brng = atan2(y, x);
    brng = rad2deg(brng);
    if (brng < 0) brng += 360.0;
    return brng;
}

static double distance_meters(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // Earth radius in meters
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLon / 2.0) * sin(dLon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

static double angle_diff(double a, double b)
{
    double diff = fmod(a - b + 540.0, 360.0) - 180.0;
    return diff;
}

GPS_Route *LOS_Navigate(
    GPS_Route *prev_wp,
    GPS_Route *curr_wp,
    double gps_lat,
    double gps_lon,
    double heading_deg,
    double ref_lat,
    double ref_lon,
    LOS_Output_t *los_out)
{
    // --- Check if route is finished ---
    if (curr_wp == NULL) {
        los_out->route_complete = true;
        los_out->waypoint_reached = true;
        los_out->desired_bearing = 0;
        los_out->heading_error = 0;
        los_out->distance_to_wp = 0;
        return NULL;
    }

    // --- Compute distance to current target (B) ---
    double dist_to_B = distance_meters(gps_lat, gps_lon, curr_wp->latitude, curr_wp->longitude);
    los_out->distance_to_wp = dist_to_B;

    // --- Check if we reached current waypoint ---
    if (dist_to_B < ARRIVAL_RADIUS_M) {
        los_out->waypoint_reached = true;

        if (curr_wp->Next_point == NULL) {
            los_out->route_complete = true;
            return NULL;  // End of route
        }
        // Advance to next waypoint
        return curr_wp->Next_point;
    }

    los_out->waypoint_reached = false;
    los_out->route_complete = false;

    // --- If there's no previous waypoint (start of route), just go directly to B ---
    if (prev_wp == NULL) {
        los_out->desired_bearing = bearing_deg(gps_lat, gps_lon, curr_wp->latitude, curr_wp->longitude);
        los_out->heading_error = angle_diff(los_out->desired_bearing, heading_deg);
        return curr_wp;
    }

    // --- Convert A, B, R to local x,y coordinates ---
    double Ax, Ay, Bx, By, Rx, Ry;
    latlon_to_xy(ref_lat, ref_lon, prev_wp->latitude, prev_wp->longitude, &Ax, &Ay);
    latlon_to_xy(ref_lat, ref_lon, curr_wp->latitude, curr_wp->longitude, &Bx, &By);
    latlon_to_xy(ref_lat, ref_lon, gps_lat, gps_lon, &Rx, &Ry);

    // --- Compute the line-of-sight (A->B vector) ---
    double ABx = Bx - Ax;
    double ABy = By - Ay;
    double AB_len = sqrt(ABx*ABx + ABy*ABy);
    if (AB_len < 1e-6) AB_len = 1e-6; // avoid divide by zero

    // --- Project rover position R onto line A-B ---
    double t = ((Rx - Ax)*ABx + (Ry - Ay)*ABy) / (AB_len*AB_len);

    // --- Compute lookahead point T ---
    double lookahead_t = t + (LOOKAHEAD_DISTANCE_M / AB_len);
    if (lookahead_t > 1.0) lookahead_t = 1.0; // don’t go beyond B, clamping 
    double Tx = Ax + lookahead_t * ABx;
    double Ty = Ay + lookahead_t * ABy;

    // --- Compute bearing from R to T ---
    double T_lat = ref_lat + (Ty / 6371000.0) * (180.0 / M_PI);
    double T_lon = ref_lon + (Tx / (6371000.0 * cos(deg2rad(ref_lat)))) * (180.0 / M_PI);
    los_out->desired_bearing = bearing_deg(gps_lat, gps_lon, T_lat, T_lon);

    // --- Compute heading error ---
    los_out->heading_error = angle_diff(los_out->desired_bearing, heading_deg);

    return curr_wp;
}

void onGPSupdate(double gps_lat, double gps_lon, double heading_deg)
{
    // Get route head
    if (curr_wp == NULL)
        curr_wp = Route_Pointer_Request();  // start at first waypoint

    current_LOS_waypoint_number = (curr_wp != NULL) ? curr_wp->nodeNumber : -1;

    curr_wp = LOS_Navigate(
        prev_wp,
        curr_wp,
        gps_lat,
        gps_lon,
        heading_deg,
        ref_lat,  // reference latitude (approx local)
        ref_lon,  // reference longitude
        &los
    );

    if (los.route_complete) {
        printf("Route complete!\n");
        hold_LOS_at_waypoint = true; // Reset hold flag
        return;
    }

    if (los.waypoint_reached) {
        // Advance prev_wp pointer
        hold_LOS_at_waypoint = true;
        osDelay(2000); // Hold for 2 seconds at waypoint
        hold_LOS_at_waypoint = false;
        prev_wp = curr_wp;
        // curr_wp already advanced inside LOS_Navigate
    }

    los_desiredheading = los.desired_bearing;

    // Use los.desired_bearing and los.heading_error in PID
    char buffer[50];
    sprintf(buffer, "Desired LOS heading: %.1f", los.desired_bearing);
    UART_puts(buffer);

}

void startRoute()
{
    GPS_Route *head = Route_Pointer_Request();
    ref_lat = head->latitude;
    ref_lon = head->longitude;
}

void LOS_caller(void *argument)
{
    while(1)
    {   
        if(enable_LOS_algo)
        {
            UART_puts("LOS waiting for notification \r\n");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for GPS update notification
            UART_puts("LOS notified by GPS task\r\n");

            startRoute();
            getlatest_GNRMC(&gnrmc_los);
            double gps_lat = convert_decimal_degrees(gnrmc_los.latitude, &gnrmc_los.NS_ind);
            double gps_lon = convert_decimal_degrees(gnrmc_los.longitude, &gnrmc_los.EW_ind);
            double heading_deg;
            getlatestHeading(&heading_deg);
            onGPSupdate(gps_lat, gps_lon, heading_deg);

        }
        osDelay(1000); // Run every 1000 ms
    }
}