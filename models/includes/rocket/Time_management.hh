#ifndef __Time_management_HH__
#define __Time_management_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Time Management Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/time.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "aux/global_constants.hh"
#include "aux/aux.hh"
#include <ctime>
#include "sim_services/include/simtime.h"
#include "Environment.hh"
#include <iomanip>

struct GPS		/* GPS Weeks and Second of Week */
{
	unsigned int Week;		/* GPS week */
	double		 SOW;			/* GPS seconds of week */
} ;

struct CALDATE	/* Calendar Date */
{
	unsigned int Year;	/* Year. */
	unsigned int Month;	/* Month. */
	unsigned int Day;	/* Day.  */
	unsigned int Hour;	/* Hours. */
	unsigned int Min;	/* Minutes.  */
	double Sec;			/* Seconds.  */
	unsigned int DOY;	/* Day of year. */
} ;

struct CUC		/* CCSDS Unsegmented time Code */
{
	unsigned char C1;	/* Coarse time part */
	unsigned char C2;
	unsigned char C3;
	unsigned char C4;
	unsigned char F1;	/* Fine time part */
	unsigned char F2;
} ;

// #include "GPS_constellation.hh"
class time_management{

	TRICK_INTERFACE(time_management);
	friend class Environment;
	friend class GPS_constellation;
	friend class INS;

	public:
		time_management();
		time_management(const time_management &other);
		~time_management(){};
		void load_start_time(unsigned int Year, unsigned int DOY, unsigned int Hour, unsigned int Min, unsigned int Sec);
		void dm_time();/* convert simulation time to gps time */
		
	private:

		GPS gpstime;
		GPS utctime;
		CALDATE caldate;

		double Julian_Date;





		void jddate(int jd,int i,int j,int k);
		void doy2dom(unsigned int year, unsigned int doy, unsigned int *m, unsigned int *dom); /* Convert Day of year to Day of month */
		void mjd_to_gps(double mjd, GPS *gpsptr); /* Transform MJD to GPS time format ONLY! No GPS to UTC correction */
		void caldoy_to_gps(CALDATE *calptr, GPS *gpsptr);
		void mjd_to_caldate(double mjd, CALDATE *calptr);/* Create a CalDate using MJD. From Montenbruck C++ code. */
		void gps_to_caldate(GPS *gpsptr, CALDATE *calptr);/* Leap seconds have *NOT* been taken into account */
		void gps_increment(GPS *gpsptr, double sec);
		void increment(GPS *gpsptr, CALDATE *calptr, double sec);
		void gps_to_utc(GPS *gpsptr, CALDATE *calptr);/* Leap seconds have been taken into account */
		void utc_to_gps(CALDATE *calptr, GPS *gpsptr);
		void unix_to_gps(time_t unix_time, GPS *gpsptr);
		void cuc_to_gps(CUC *cucptr, GPS *gpsptr);
		void gps_to_cuc(GPS *gpsptr, CUC *cucptr);
		void gps_diff(GPS *gpsptr1, GPS *gpsptr2, GPS *gpsptr_out);/* Calculate 2 GPS time difference */


		int tai_utc(double mjd); /*Return the difference between TAI and UTC (known as leap seconds)*/
		int jd(int i,int j,int k);
		int ymd2day(int year,int month,int dayOfMonth);
		int caldate_to_jdn(CALDATE *calptr);

		double gps_utc_diff(GPS *gpsptr);/* Calculate difference between GPS time & UTC time */
		double caldate_to_mjd(CALDATE *calptr);
		double time_fmod(double a, double b);
		double gps_to_mjd(GPS *gpsptr); /* Leap seconds have *NOT* been taken into account */

		unsigned int day2doy(unsigned int year, unsigned int month, unsigned int mday); /* Convert YY-MM-DD to Day of Year */

		time_t gps_to_unix(GPS *gpsptr);

		float cuc_diff(CUC *cucptr1, CUC *cucptr2);

		void utctime2gpstime(const GPS *utcptr, GPS *gpsptr);
};




#endif
