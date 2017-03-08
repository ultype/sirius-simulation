#include "rocket/Time_management.hh"

time_management::time_management()
{
    caldate.Year = 0;
    caldate.DOY = 0;
    caldate.Hour = 0;
    caldate.Min = 0;
    caldate.Sec = 0;
    caldate.Day = 0;
    caldate.Month = 0;

    gpstime.Week = 0;
    gpstime.SOW = 0;
}

time_management::time_management(const time_management &other)
{
    this->gpstime = other.gpstime;
    this->utctime = other.utctime;
    this->caldate = other.caldate;
    this->Julian_Date = other.Julian_Date;
}

double time_management::time_fmod(double a, double b)
{
	double temp, absu;

	absu = (double)fabs(a);
	temp = absu - b * (int)(absu/b);
	if (temp  < 0.0) temp = temp + fabs(b); 
	if (a < 0.0) temp =-temp;
	return temp;
}	/* End of dm_fmod */


/* Convert YY-MM-DD to Day of Year */
unsigned int time_management::day2doy(unsigned int year, unsigned int month, unsigned int mday)
{
    unsigned int regu_month_day[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unsigned int leap_month_day[12] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    unsigned int yday = 0;

    /* check for leap year */
    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0))
	{
        yday = leap_month_day[month - 1] + mday;
    }
    else
	{
		yday = regu_month_day[month - 1] + mday;
    }

    return yday;
}	/* End of day2doy */


/* Convert Day of year to Day of month */
void time_management::doy2dom(unsigned int year, unsigned int doy, unsigned int *m, unsigned int *dom)
{
	unsigned int month_array[12]={ 31, 28, 31, 30, 31, 30 ,31, 31, 30, 31, 30, 31 };
	unsigned int month_array_leap_year[12]={ 31, 29, 31, 30, 31, 30 ,31, 31, 30, 31, 30, 31 };
    unsigned int month = 0;

    /* check for leap year */
    if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
	{
		while (doy > month_array_leap_year[month])
		{
			doy = doy - month_array_leap_year[month];
			month = month + 1;
		}
	}
	else
	{
		while (doy > month_array[month])
		{
			doy = doy - month_array[month];
			month = month + 1;
		}
	}

	*m   = month + 1;
    *dom = doy;
}


/* Return the difference between TAI and UTC (known as leap seconds).      */
/* Values from the USNO website: ftp://maia.usno.navy.mil/ser7/leapsec.dat */
/*                               ftp://maia.usno.navy.mil/ser7/tai-utc.dat */
/* Check IERS Bulletin C. (International Earth Rotation and Reference      */
/* Systems Service) http://www.iers.org/  */
/* @param mjd Modified Julian Date        */
/* @return number of leaps seconds.       */

int time_management::tai_utc(double mjd)
{
    if (mjd < 0.0)
	{
        cout<<"MJD before the beginning of the leap sec table"<<endl;
        return 0;
    }

    if ((mjd >=41317.0)&&(mjd < 41499.0)) return 10;  /* January 1, 1972 */
    if ((mjd >=41499.0)&&(mjd < 41683.0)) return 11;  /* July 1, 1972    */
    if ((mjd >=41683.0)&&(mjd < 42048.0)) return 12;  /* January 1, 1973 */
    if ((mjd >=42048.0)&&(mjd < 42413.0)) return 13;  /* January 1, 1974 */
    if ((mjd >=42413.0)&&(mjd < 42778.0)) return 14;  /* January 1, 1975 */
    if ((mjd >=42778.0)&&(mjd < 43144.0)) return 15;  /* January 1, 1976 */
    if ((mjd >=43144.0)&&(mjd < 43509.0)) return 16;  /* January 1, 1977 */
    if ((mjd >=43509.0)&&(mjd < 43874.0)) return 17;  /* January 1, 1978 */
    if ((mjd >=43874.0)&&(mjd < 44239.0)) return 18;  /* January 1, 1979 */
    if ((mjd >=44239.0)&&(mjd < 44786.0)) return 19;  /* January 1, 1980 */
    if ((mjd >=44786.0)&&(mjd < 45151.0)) return 20;  /* July 1, 1981    */
    if ((mjd >=45151.0)&&(mjd < 45516.0)) return 21;  /* July 1, 1982    */
    if ((mjd >=45516.0)&&(mjd < 46247.0)) return 22;  /* July 1, 1983    */
    if ((mjd >=46247.0)&&(mjd < 47161.0)) return 23;  /* July 1, 1985    */
    if ((mjd >=47161.0)&&(mjd < 47892.0)) return 24;  /* January 1, 1988 */
    if ((mjd >=47892.0)&&(mjd < 48257.0)) return 25;  /* January 1, 1990 */
    if ((mjd >=48257.0)&&(mjd < 48804.0)) return 26;  /* January 1, 1991 */
    if ((mjd >=48804.0)&&(mjd < 49169.0)) return 27;  /* July 1st, 1992  */
    if ((mjd >=49169.0)&&(mjd < 49534.0)) return 28;  /* July 1, 1993    */
    if ((mjd >=49534.0)&&(mjd < 50083.0)) return 29;  /* July 1, 1994    */
    if ((mjd >=50083.0)&&(mjd < 50630.0)) return 30;  /* January 1, 1996 */
    if ((mjd >=50630.0)&&(mjd < 51179.0)) return 31;  /* July 1, 1997    */
    if ((mjd >=51179.0)&&(mjd < 53736.0)) return 32;  /* January 1, 1999 */
    if ((mjd >=53736.0)&&(mjd < 54832.0)) return 33;  /* January 1, 2006 */
    if ((mjd >=54832.0)&&(mjd < 56109.0)) return 34;  /* January 1, 2009 */
    if ((mjd >=56109.0)&&(mjd < 57204.0)) return 35;  /* July 1, 2012 */
    if ((mjd >=57204.0)&&(mjd < 57754.0)) return 36;  /* July 1, 2015 */
    if (mjd >=57754.0) return 37;  /* January 1, 2017 */

    cout<<"Input MJD out of bounds"<<endl;
    return 0;
}

/*
*
*    This function converts a calendar date to the corresponding Julian
*    day starting at noon on the calendar date.  The algorithm used is
*    from Van Flandern and Pulkkinen, Ap. J. Supplement Series 41, 
*    November 1979, p. 400.
*
*
*	Arguments
*     
*     	Name    Type 	I/O 	Description
*     	----	---- 	--- 	-----------
*     	i		I*4  	 I 		Year - e.g. 1970
*     	j       I*4  	 I  	Month - (1-12)
*     	k       I*4  	 I  	Day  - (1-31)
*     	jd      I*4  	 O  	Julian day
*
*     external references
*     -------------------
*      none
*
*
*     Written by Frederick S. Patt, GSC, November 4, 1992
*
*/

int time_management::jd(int i,int j,int k)
{
	int jd;

	jd = 367*i - 7*(i+(j+9)/12)/4 + 275*j/9 + k + 1721014;

	/* This additional calculation is needed only for dates outside of the */ 
	/* period March 1, 1900 to February 28, 2100 */
	/* jd = jd + 15 - 3*((i+(j-9)/7)/100+1)/4 */
    
	return jd;
}


/* Transform MJD to GPS time format ONLY! No GPS to UTC correction */
void time_management::mjd_to_gps(double mjd, GPS *gpsptr)
{
	double imjd = floor(mjd);
    double fmjd = mjd - floor(mjd);

    gpsptr->Week = (unsigned int) ((imjd - JAN61980) / 7.0);
    gpsptr->SOW = ( (imjd - JAN61980) - gpsptr->Week * 7 + fmjd ) * SEC_PER_DAY;
}	/* mjd_to_gps */

void time_management::caldoy_to_gps(CALDATE *calptr, GPS *gpsptr)
{
    double mjd;
    double fmjd;

	/* caldoy -> mjd */
    mjd = ((calptr->Year - 1901)/4)*1461 + ((calptr->Year - 1901)%4)*365 + calptr->DOY - 1 + JAN11901;
    fmjd = ((calptr->Sec/60.0 + calptr->Min)/60.0 + calptr->Hour)/24.0;

    /*printf("mjd = %.0f  fmjd = %.15f\n", mjd, fmjd); */

	/* mjd -> gps */
    gpsptr->Week = (unsigned int) ((mjd - JAN61980) / 7.0);
    gpsptr->SOW = ( (mjd - JAN61980) - gpsptr->Week * 7 + fmjd ) * SEC_PER_DAY;
}

double time_management::caldate_to_mjd(CALDATE *calptr)
{
    int    MjdMidnight;
    double  FracOfDay, mjd;
    int     b, temp;
	/*double jd;*/
    if (calptr->Month <= 2)
	{
        calptr->Month += 12;
        --(calptr->Year);
    }

    if ( (10000 * calptr->Year + 100 * calptr->Month + calptr->Day) <= 15821004 )
	{	/* For a date in the Julian calendar , up to 1582 October 4 */
        b = -2 + (int) ((calptr->Year + 4716) /4) - 1179;
    }
    else
	{	/* Gregorian calendar is used from 1582 October 15 onwards */
        b = (int) (calptr->Year / 400) - (int) (calptr->Year / 100) + (int) (calptr->Year / 4);
    }

    temp = (int) (30.6001 * (calptr->Month + 1));

    MjdMidnight = 365 * calptr->Year - 679004 + b + temp + calptr->Day;
    /*FracOfDay   = (calptr->Hour + calptr->Min/60.0 + calptr->Sec/3600.0) / 24.0; */
	FracOfDay   = (calptr->Hour * 3600 + calptr->Min * 60.0 + calptr->Sec) / SEC_PER_DAY;

    mjd = (double) MjdMidnight + FracOfDay;

    /*printf("mjd = %.15f\n", mjd); */

	/*** Algorithm provided by Vallado's book 3rd Edition ***/
	/*** page 189, Algorithm 14, verified same result as  ***/
	/*** the above algorithm.                             ***/
	/***
	double jd;

	jd = 367.0 * calptr->Year -
		 floor((7 * (calptr->Year + floor((calptr->Month + 9) / 12.0))) * 0.25) +
		 floor( 275 * calptr->Month / 9.0 ) +
		 calptr->Day + 1721013.5 +
		 ((calptr->Sec / 60.0 + calptr->Min) / 60.0 + calptr->Hour) / 24.0; ***/
		 /* - 0.5 * sgn(100.0*year + mon - 190002.5) + 0.5; */

	/***
	mjd = jd - 2400000.5;	
    ***/

    return mjd;
}	/* caldate_to_mjd */

int time_management::caldate_to_jdn(CALDATE *calptr)
{
	/*int a, m; */

	double y, c;
    /***
    	a = (int) ((14 - calptr->Month) / 12);
    	y = calptr->Year + 4800 - a;
    	m = + 12 * a - 3;

    	if ( (10000L * calptr->Year + 100L * calptr->Month + calptr->Day) <= 15821004L )
    	{
    		return (calptr->Day + (int) ((153 * m + 2) / 5) + 365 * y + (int) (y/4) - (int) (y/100) + (int) (y/400) - 32045);
    	}
    	else
    	{
    		return (calptr->Day + (int) ((153 * m + 2) / 5) + 365 * y + (int) (y/4) - 32083);
    	}
    ***/

	/* from http:www.merlyn.demon.co.uk/programs/daterate.pas */
	y = 1.0 * calptr->Year + (1.0 * calptr->Month - 2.85) / 12;
	if ( (10000L * calptr->Year + 100L * calptr->Month + calptr->Day) <= 15821004L )
	{
		c = 0.75 * 2;
	}
    else
	{
		c = 0.75 * floor(y / 100);
	}

	return ( (int) (floor(floor(367 * y) - 1.75 * floor(y) + calptr->Day) - c) + 1721115 );
}	/* caldate_to_jdn */

/* Create a CalDate using MJD. From Montenbruck C++ code. */
/* Astronomy on the Personal Computer (Springer, ISBN: 0387577009) */

void time_management::mjd_to_caldate(double mjd, CALDATE *calptr)
{
    long    a,b,c,d,e,f, temp;
    double  Hours, x;

    a = (long)(mjd+2400001.0);

    if ( a < 2299161 )
	{
        b = 0;
        c = a + 1524;
    }
    else
	{
        b = (long) ((a - 1867216.25) / 36524.25);
        c = a +  b - (b/4) + 1525;
    }

    d     = (long)( (c - 122.1) / 365.25 );
    e     = 365 * d + d / 4;
    f     = (long) ( (c - e) / 30.6001 );

    temp = (long) (30.6001 * f);
    calptr->Day   = (int) (c - e - temp);
    temp = (long) (f / 14);
    calptr->Month = (int) (f - 1 - 12 * temp);
    temp = (long) ((7 + calptr->Month) / 10);
    calptr->Year  = (int) (d - 4715 - temp);

    Hours = 24.0 * (mjd - floor(mjd));

    calptr->Hour = (int) Hours;
    x = (Hours - calptr->Hour) * 60.0;
    calptr->Min = (int) x;
    calptr->Sec = (x - calptr->Min) * 60.0;
    calptr->DOY = day2doy(calptr->Year, calptr->Month, calptr->Day);
}

/* Leap seconds have *NOT* been taken into account */
/* *gpsptr is UTC TIME in GPS format */
void time_management::gps_to_caldate(GPS *gpsptr, CALDATE *calptr)
{
    // *****
    //     double mjd;

    //     mjd = gpsptr->Week * 7.0 + gpsptr->SOW / SEC_PER_DAY + JAN61980
    // 		+ fmod(gpsptr->SOW, SEC_PER_DAY) / SEC_PER_DAY;

    // 	mjd_to_caldate(mjd, calptr);
    // ****

    double mjd, fmjd, days_since_jan1_1901;
    int delta_yrs, num_four_yrs, years_so_far, days_left;

	/* Convert GPS time to MJD */
    mjd  = gpsptr->Week * 7.0 + floor(gpsptr->SOW / SEC_PER_DAY) + JAN61980;
    fmjd = time_fmod(gpsptr->SOW, SEC_PER_DAY) / SEC_PER_DAY;

    days_since_jan1_1901 = mjd - JAN11901;
    num_four_yrs = (int) (days_since_jan1_1901 / 1461);  /* 4 years = 1461 days */
    years_so_far = 1901 + 4 * num_four_yrs;
    days_left = (long) (days_since_jan1_1901 - 1461 * num_four_yrs);
    delta_yrs = days_left / 365 - days_left / 1460;

    calptr->Year  = (unsigned int) (years_so_far + delta_yrs);
    calptr->DOY   = (unsigned int) (days_left - 365 * delta_yrs + 1);
    /*calptr->Month = doy2month(calptr->Year, calptr->DOY); */
    /*calptr->Day   = doy2day(calptr->Year, calptr->DOY); */
	doy2dom(calptr->Year, calptr->DOY, &calptr->Month, &calptr->Day);
    calptr->Hour  = (int) (fmjd * 24.0);
    calptr->Min   = (int) (fmjd * 1440.0 - calptr->Hour * 60.0);
    calptr->Sec   = fmjd * 86400.0 - calptr->Hour * 3600.0 - calptr->Min * 60.0;

}

/* Leap seconds have *NOT* been taken into account */
/* Check Output :  http://www.csgnetwork.com/julianmodifdateconv.html */
/* *gpsptr is UTC TIME in GPS format */
double time_management::gps_to_mjd(GPS *gpsptr)
{
    double mjd = gpsptr->Week * 7.0 + gpsptr->SOW/SEC_PER_DAY + JAN61980; /* 44244 */
    return mjd;
}

void time_management::gps_increment(GPS *gpsptr, double sec)
{
    double temp = gpsptr->SOW + sec;
    /*double max_sow = 604800.0; */

    if (fabs(sec) > SEC_PER_WEEK)
	{
        printf("Sorry, Can't increment time by >= 1 week\n");
        return;
    }

    if (temp < 0.0)
	{
        gpsptr->Week = gpsptr->Week - 1;
        gpsptr->SOW = SEC_PER_WEEK + temp;
    }

    if (temp >= SEC_PER_WEEK)
	{
        gpsptr->Week = gpsptr->Week + 1;
        gpsptr->SOW  = temp - SEC_PER_WEEK;
    }

    if ((temp >= 0.0)&&(temp < SEC_PER_WEEK))
	{
        gpsptr->SOW = temp;
    }
}	/* End of gps_increment */

void time_management::increment(GPS *gpsptr, CALDATE *calptr, double sec)
{
    CALDATE tmp_cal;

    gps_increment(gpsptr, sec);
    gps_to_caldate(gpsptr, &tmp_cal);

    calptr->Year  = tmp_cal.Year;
    calptr->Month = tmp_cal.Month;
    calptr->Day   = tmp_cal.Day;
    calptr->Hour  = tmp_cal.Hour;
    calptr->Min   = tmp_cal.Min;
    calptr->Sec   = tmp_cal.Sec;
    calptr->DOY   = tmp_cal.DOY;
}

/* Leap seconds have been taken into account */
void time_management::gps_to_utc(GPS *gpsptr, CALDATE *calptr)
{
    double mjd = gps_to_mjd(gpsptr);
    int utc_gps = TAI_GPS - tai_utc(mjd);

    increment(gpsptr, calptr, utc_gps);
}

void time_management::utc_to_gps(CALDATE *calptr, GPS *gpsptr)
{
    double mjd = caldate_to_mjd(calptr);
    int gps_utc = tai_utc(mjd) - TAI_GPS;

	mjd_to_gps(mjd, gpsptr);
    gps_increment(gpsptr, gps_utc);
}

time_t time_management::gps_to_unix(GPS *gpsptr)
{
    time_t unixtime;
	double mjd;

    mjd = gps_to_mjd(gpsptr);
	unixtime = (long) ((mjd - UNIX_MJD) * SEC_PER_DAY);
	return unixtime;
}

void time_management::unix_to_gps(time_t unix_time, GPS *gpsptr)
{
  double mjd;

  mjd = unix_time/SEC_PER_DAY + UNIX_MJD;

  mjd_to_gps(mjd, gpsptr);
}	/* End of unix_to_gps */

/* Convert CUC ( CCSDS Unsegmented time Code) to GPS time */
void time_management::cuc_to_gps(CUC *cucptr, GPS *gpsptr)
{
	double integral_part, fractional_part;

	integral_part = cucptr->C1 * 16777216.0 + cucptr->C2 * 65536.0
					+ cucptr->C3 * 256.0 + cucptr->C4;
	fractional_part = cucptr->F1 * 0.00390625 + cucptr->F2 * 0.0000152587890625;

	/* The difference between GPS epoch (Jan. 6 1980) and the beginning */
	/* of the CUC time (Jan. 6 1981) is 52 weeks + 2 days (172800 secs) */
	/* Note: 1980 is a leap year */
	gpsptr->Week = (unsigned int) (floor(integral_part / SEC_PER_WEEK)) + 52;
	gpsptr->SOW  = time_fmod(integral_part, SEC_PER_WEEK) + fractional_part + 172800.0;
	if (gpsptr->SOW >= SEC_PER_WEEK)
	{
		gpsptr->Week = (unsigned int) (gpsptr->Week + floor(gpsptr->SOW / SEC_PER_WEEK));
		gpsptr->SOW  = time_fmod(gpsptr->SOW, SEC_PER_WEEK);
	}
}	/* End of cuc_to_gps */

void time_management::gps_to_cuc(GPS *gpsptr, CUC *cucptr)
{
	double total_sec, C1_sec, C2_sec, C3_sec;

	/* GPS epoch : Jan. 6 1980 */
	/* CUC epoch : Jan. 6 1981 */
	/* Difference between CUC and GPS is 52 weeks and 2 days (172800 secs) */
	if (((gpsptr->Week == 52.0) && (gpsptr->SOW >= 172800.0)) ||
		 (gpsptr->Week > 52.0) )
	{
		if (gpsptr->SOW < 172800.0)
		{
			total_sec = (gpsptr->Week - 53.0) * SEC_PER_WEEK + (SEC_PER_WEEK - 172800.0)
				+ gpsptr->SOW;
		}
		else
		{
			total_sec = (gpsptr->Week - 52.0) * SEC_PER_WEEK + (gpsptr->SOW - 172800.0);
		}

		cucptr->C1 = (unsigned char) (total_sec / 16777216.0);
		C1_sec = cucptr->C1 * 16777216.0;	/* 256^3 */

		cucptr->C2 = (unsigned char) ((total_sec - C1_sec) / 65536.0);
		C2_sec = cucptr->C2 * 65536.0;		/* 256^2 */

		cucptr->C3 = (unsigned char) ((total_sec - C1_sec - C2_sec) / 256.0);
		C3_sec = cucptr->C3 * 256.0;

		cucptr->C4 = (unsigned char) (floor(total_sec - C1_sec - C2_sec - C3_sec));

		total_sec = total_sec - C1_sec - C2_sec - C3_sec - cucptr->C4;

		cucptr->F1 = (unsigned char) (total_sec / 0.00390625);
		cucptr->F2 = (unsigned char) ((total_sec - cucptr->F1 * 0.00390625) / 0.0000152587890625);
	}
	else
	{
		printf("CUC time is not available! week=%d\n", gpsptr->Week);
	}
}	/* End of gps_to_cuc */

float time_management::cuc_diff(CUC *cucptr1, CUC *cucptr2)
{
	float diff;

	diff = (float) ( (cucptr2->C1 - cucptr1->C1) * 16777216.0	/* 256^3 */
			+ (cucptr2->C2 - cucptr1->C2) * 65536.0				/* 256^2 */
			+ (cucptr2->C3 - cucptr1->C3) * 256.0
			+ (cucptr2->C4 - cucptr1->C4)
			+ (cucptr2->F1 - cucptr1->F1) * 0.00390625			/* 256^(-1) */
			+ (cucptr2->F2 - cucptr1->F2) * 0.0000152587890625 );	/*256^(-2) */

	return diff;
}	/* cuc_diff */


/* Calculate 2 GPS time difference */
void time_management::gps_diff(GPS *gpsptr1, GPS *gpsptr2, GPS *gpsptr_out)
{
	/* substracts the week number */
	gpsptr_out->Week = gpsptr1->Week - gpsptr2->Week;

	/* substracts the number of elapsed second since the begining of the */
	/* current week and the number of 1/256th of second since last second */
	gpsptr_out->SOW = gpsptr1->SOW - gpsptr2->SOW;

	/* updates the week number if the number of elapsed second since the */
	/* beginning of the current week is less than 0 */
	if (gpsptr_out->SOW < 0.0)
	{
		gpsptr_out->SOW  = gpsptr_out->SOW + SEC_PER_WEEK;
        gpsptr_out->Week = gpsptr_out->Week - 1;
	}
}	/* End of gps_diff */

/* Calculate difference between GPS time & UTC time */
/* Input : GPS time*/
/* Output: GPS - UTC, Note: UTC < GPS */
double time_management::gps_utc_diff(GPS *gpsptr)
{
    double mjd = gps_to_mjd(gpsptr);
    double gps_utc = (double) (tai_utc(mjd) - TAI_GPS);

	return (gps_utc);
}

void time_management::dm_time()/* convert simulation time to gps time */
{
	
	/* Get current GPS time */
	/* deduct DM_dt because PV comes from dm_att() output of previous cycle */
	gps_increment(&utctime, 0.001);

	gpstime.Week = utctime.Week;
	gpstime.SOW  = utctime.SOW;

	/* Convert UTC time to GPS time, UTC < GPS */
	gps_increment(&gpstime, gps_utc_diff(&gpstime));

	/* Convert UTC time to MJD & JD time */
	Julian_Date = gps_to_mjd(&utctime) + 2400000.5;	/* DM_Julian_Date : Julian Date */
    gps_to_caldate(&utctime, &caldate); //Convert UTC(GPS form) time to calendar Date
    gps_to_utc(&gpstime, &caldate);
    //cout<<setprecision(10)<<"gps week: "<<gpstime.Week<<'\t'<<"gps sow :"<<gpstime.SOW<<'\t'<<"utc week:"<<utctime.Week<<'\t'<<"utc sow :"<<utctime.SOW<<endl;
    // cout<<setprecision(10)<<"Year: "<<caldate.Year<<'\t'<<"Month: "<<caldate.Month<<'\t'<<"Day: "<<caldate.Day<<'\t'<<"Min: "<<caldate.Min<<'\t'<<"Sec: "<<caldate.Sec<<endl;
    //cout<<setprecision(20)<<"Julian_Date: "<<Julian_Date<<endl;
}

int time_management::ymd2day(int year,int month,int dayOfMonth)  
{
    int dayOfYear;

    int regu_month_day[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int leap_month_day[12] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};

    /* check for leap year */
    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) 
        {   
        dayOfYear = leap_month_day[month - 1] + dayOfMonth;
    }   
    else
        {   
                dayOfYear = regu_month_day[month - 1] + dayOfMonth;
    }   

        return (dayOfYear);
}       /* ymd2day */

void time_management::jddate(int jd,int i,int j,int k)
{
        double l, n;

        l = jd + 68569;
        n = 4*l/146097;
        l = l - (146097*n + 3)/4;
        i = (int) (4000*(l+1)/1461001);
        l = l - 1461*i/4 + 31;
        j = (int) (80*l/2447);
        k = (int) (l - 2447*j/80);
        l = j/11;
        j = (int) (j + 2 - 12*l);
        i = (int) (100*(n-49) + i + l);
}
//Load simulation start time in Calender date form and UTC time and convert into gps time form
void time_management::load_start_time(unsigned int Year, unsigned int DOY, unsigned int Hour, unsigned int Min, unsigned int Sec)
{
    this->caldate.Year = Year;
    this->caldate.DOY = DOY;
    this->caldate.Hour = Hour;
    this->caldate.Min = Min;
    this->caldate.Sec = Sec;

    caldoy_to_gps(&caldate, &utctime);//convert into gps time
    /* Convert UTC time to MJD & JD time */
    Julian_Date = gps_to_mjd(&utctime) + 2400000.5; /* DM_Julian_Date : Julian Date */

}
