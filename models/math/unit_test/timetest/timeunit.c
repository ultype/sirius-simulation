#include "time_utility_c.h"

int main() {
	GPS_TIME gps;
	UTC_TIME utc;
	double mjd;
	// gps.week = 2012;
	// gps.SOW = 172818 + 86400;
	utc.year = 2018;
	utc.month = 8;
	utc.day = 2;
	utc.hour = 6;
	utc.min = 13;
	utc.sec = 18.0;
	printf("*********Time service test***********\n");
	printf("Input: 8-2-2018 6:13:18\n");
	printf("Result :\n");

	printf("***Calendar date to Day of year***\n");
	UTC_CALENDAR_DATE_2_DOY(&utc);
	printf("%d-%d\n", utc.year, utc.DOY);
	printf("\n");

	printf("***UTC time to MJD***\n");
	UTC_TIME_2_Modified_julian_date(utc, &mjd);
	printf("MJD = %f\n", mjd);
	printf("\n");

	printf("***UTC time to GPS time***\n");
	UTC_TIME_2_GPS_TIME(utc, &gps);
	printf("GPS TIME: Week:%d\tSOW:%f\n", gps.week, gps.SOW);
	printf("\n");

	printf("***GPS TIME to MJD***\n");
	GPS_TIME_2_Modified_julian_date(gps, &mjd);
	printf("MJD = %f\n", mjd);
	printf("\n");

	printf("***MJD to UTC time***\n");
	MJD_2_UTC_TIME(mjd, &utc);
	printf("UTC TIME:%d-%d-%d %d:%d:%f\n", utc.month, utc.day, utc.year, utc.hour, utc.min, utc.sec);
	printf("\n");

	printf("***GPS time tp UTC time***\n");
	GPS_TIME_2_UTC_TIME(gps, &utc);
	printf("UTC TIME:%d-%d %d:%d:%f\n", utc.year, utc.DOY, utc.hour, utc.min, utc.sec);
	printf("\n");

	printf("***MJD to GPS time***\n");
	MJD_2_GPS_TIME(mjd, &gps);
	printf("GPS TIME: Week:%d\tSOW:%f\n", gps.week, gps.SOW);
	printf("\n");

	printf("***UTC time DOY to Calendar date***\n");
	UTC_TIME_DOY_2_CALENDAR_DATE(utc.year, utc.DOY, &utc);
	printf("CALENDAR DATE: %d-%d-%d\n", utc.month, utc.day, utc.year);
	
	return 0;
}
