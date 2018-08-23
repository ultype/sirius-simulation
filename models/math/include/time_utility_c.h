#ifndef __TIME_UTIL_C_H__
#define __TIME_UTIL_C_H__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Helper functions related to time)
LIBRARY DEPENDENCY:
      ((../src/time_utility_c.c))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif
    
typedef struct _GPS_TIME {
    uint32_t week;
    double SOW;
}GPS_TIME;

typedef struct _UTC_TIME {
    uint32_t year;
    uint32_t DOY;
    uint32_t month;
    uint32_t day; 
    uint32_t hour;
    uint32_t min; 
    double sec;
}UTC_TIME;

typedef struct _CCSDS_CUC {
    uint8_t C1;
    uint8_t C2;
    uint8_t C3;
    uint8_t C4;
    uint8_t F1;
    uint8_t F2;
}CCSDS_CUC;

int GPS_TIME_2_Modified_julian_date(GPS_TIME in, double *modified_julian_date);
uint32_t tai_leap_second(const double mjd);
int UTC_TIME_2_Modified_julian_date(UTC_TIME in, double *modified_julian_date);
int GPS_TIME_2_UTC_TIME(GPS_TIME in, UTC_TIME *out);
int MJD_2_UTC_TIME(const double mjd, UTC_TIME *out);
int UTC_TIME_2_GPS_TIME(UTC_TIME in, GPS_TIME *out);
int MJD_2_GPS_TIME(double mjd, GPS_TIME *out);
int UTC_TIME_DOY_2_CALENDAR_DATE(uint32_t year_in, uint32_t doy_in, UTC_TIME *out);
int UTC_CALENDAR_DATE_2_DOY(UTC_TIME *utc);
int MJD_2_JD(double mjd, double *jd);
int load_start_time(unsigned int Year, unsigned int DOY, unsigned int Hour, unsigned int Min, double Sec, UTC_TIME *utc_time, GPS_TIME *gps_time);
int time_add(double in, GPS_TIME *gps);
#ifdef __cplusplus
}
#endif


#endif