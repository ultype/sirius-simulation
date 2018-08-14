#include "time_utility_c.h"
/*********************************************************************
*   http://stjarnhimlen.se/comp/time.html
*   http://www.cv.nrao.edu/~rfisher/Ephemerides/times.html
*   http://maia.usno.navy.mil/search/search.html
**********************************************************************/

/** UTC Time/Julian Day/GPS Time **/
const double DJ00 = 2451545.0;
const double DJC = 36525.0;
const double DAS2R = 4.848136811095359935899141e-6; /* Arcseconds to radians */
const double D2PI = 6.283185307179586476925287; /* 2Pi */
const double DMAS2R = 4.848136811095359935899141e-9; /* Milliarcseconds to radians */
const double TURNAS = 1296000.0; /* Arcseconds in a full circle */
const double SEC_PER_DAY = 86400.0;
const double SEC_PER_WEEK = 604800.0;
const double TT_TAI = 32.184;

/* Difference between TAI and GPS */
const uint32_t TAI_GPS_DIFF = 19;
/* MJD of Unix epoch : 00:00:00 on January 1, 1970 */
const uint32_t MJD_UNIX = 40587;
/* MJD of GPS epoch : Jan. 6 1980 */
const double MJD_JAN61980 = 44244.0;
/* MJD of Jan 1, 1901 */
const double MJD_JAN11901 = 15385.0;


int GPS_TIME_2_Modified_julian_date(GPS_TIME in, double *modified_julian_date) {
    *modified_julian_date = in.week * 7.0 + in.SOW / SEC_PER_DAY + MJD_JAN61980;
    int32_t UTC_GPS_DIFF = TAI_GPS_DIFF - tai_leap_second(*modified_julian_date);

    double temp = in.SOW + UTC_GPS_DIFF;

    if (temp < 0.0) {
        in.week = in.week - 1;
        in.SOW = SEC_PER_WEEK + temp;
    } else if (temp >= SEC_PER_WEEK) {
        in.week = in.week + 1;
        in.SOW = temp - SEC_PER_WEEK;
    } else {
        in.SOW = temp;
    }

    *modified_julian_date = in.week * 7.0 + in.SOW / (double)SEC_PER_DAY + MJD_JAN61980;

    return 0;
}

// Return the difference between TAI and UTC (known as leap seconds).
// Values from the USNO website: ftp://maia.usno.navy.mil/ser7/leapsec.dat
//                               ftp://maia.usno.navy.mil/ser7/tai-utc.dat
// Check IERS Bulletin C. (International Earth Rotation and Reference
// Systems Service) http://www.iers.org/
// @param mjd Modified Julian Date
// @return number of leaps seconds.
uint32_t tai_leap_second(const double mjd) {
    if (mjd < 0.0) {
        fprintf(stderr, "MJD before the beginning of the leap sec table\n");
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

    fprintf(stderr, "Input MJD out of bounds\n");

    return 0;
}

double time_fmod(double a, double b) {
    double temp, absu;

    absu = fabs(a);
    temp = absu - b * (int32_t)(absu / b);

    if (temp  < 0.0) temp = temp + fabs(b);
    if (a < 0.0) temp =-temp;
    return temp;
}

int UTC_TIME_2_Modified_julian_date(UTC_TIME in, double *modified_julian_date) {
    int     MjdMidnight;
    double  FracOfDay;
    int     b, temp;

    if (in.month <= 2) {
        in.month = in.month + 12;
        in.year = in.year - 1;
    }

    if ((10000 * in.year + 100 * in.month + in.day) <= 15821004) {
        /* For a date in the Julian calendar , up to 1582 October 4 */
        b = -2 + (int32_t)((in.year + 4716) / 4) - 1179;
    } else {
        /* Gregorian calendar is used from 1582 October 15 onwards */
        b = (int32_t)(in.year / 400) - (int32_t)(in.year / 100) + (int32_t)(in.year / 4);
    }

    temp = (int32_t)(30.6001 * (in.month + 1));
    MjdMidnight = 365 * in.year - 679004 + b + temp + in.day;
    FracOfDay = (in.hour * 3600.0 + in.min * 60.0 + in.sec) / SEC_PER_DAY;

    *modified_julian_date = (double)MjdMidnight + FracOfDay;

    return 0;
}

int GPS_TIME_2_UTC_TIME(GPS_TIME in, UTC_TIME *out) {
    double mjd, fmjd, days_since_jan1_1901;
    int delta_yrs, num_four_yrs, years_so_far, days_left;

    GPS_TIME_2_Modified_julian_date(in, &mjd);
    int32_t UTC_GPS_DIFF = TAI_GPS_DIFF - tai_leap_second(mjd);

    double temp = in.SOW + UTC_GPS_DIFF;

    if (temp < 0.0) {
        in.week = in.week - 1;
        in.SOW = SEC_PER_WEEK + temp;
    } else if (temp >= SEC_PER_WEEK) {
        in.week = in.week + 1;
        in.SOW = temp - SEC_PER_WEEK;
    } else {
        in.SOW = temp;
    }

    mjd = in.week * 7.0 + floor(in.SOW / SEC_PER_DAY) + MJD_JAN61980;
    fmjd = time_fmod(in.SOW, SEC_PER_DAY) / SEC_PER_DAY;

    days_since_jan1_1901 = mjd - MJD_JAN11901;
    num_four_yrs = (int32_t)(days_since_jan1_1901 / 1461);  /* 4 years = 1461 days */
    years_so_far = 1901 + 4 * num_four_yrs;
    days_left = (int64_t)(days_since_jan1_1901 - 1461 * num_four_yrs);
    delta_yrs = days_left / 365 - days_left / 1460;

    out->year = (uint32_t)(years_so_far + delta_yrs);
    out->DOY = (uint32_t)(days_left - 365 * delta_yrs + 1);
    out->hour = (int32_t)(fmjd * 24.0);
    out->min = (int32_t)(fmjd * 1440 - out->hour * 60.0);
    out->sec = fmjd * 86400.0 - out->hour * 3600.0 - out->min * 60.0;

    return 0;
}

int MJD_2_UTC_TIME(const double mjd, UTC_TIME *out) {
    uint64_t a, b, c, d, e, f, temp;
    double  Hours, x;

    a = (int64_t)(mjd + 2400001.0);

    if ( a < 2299161 ) {
        c = a + 1524;
    } else {
        b = (int64_t)((a - 1867216.25) / 36524.25);
        c = a + b - (b / 4) + 1525;
    }

    d     = (int64_t)((c - 122.1) / 365.25);
    e     = 365 * d + d / 4;
    f     = (int64_t)((c - e) / 30.6001);

    temp = (int64_t)(30.6001 * f);
    out->day = (int32_t)(c - e - temp);

    temp = (int64_t)(f / 14);
    out->month = (int32_t)(f - 1 - 12 * temp);

    temp = (int64_t)((7 + out->month) / 10);
    out->year = (int32_t)(d - 4715 - temp);

    Hours = 24.0 * (mjd - floor(mjd));
    out->hour = (int32_t)(Hours);

    x = (Hours - out->hour) * 60.0;
    out->min = (int32_t)(x);
    out->sec = (x - out->min) * 60.0;

    return 0;
}

int UTC_TIME_2_GPS_TIME(UTC_TIME in, GPS_TIME *out) {
    double mjd, fmjd;

    /* DOY -> mjd */
    mjd = ((in.year - 1901) / 4) * 1461 + ((in.year - 1901) % 4) * 365 + in.DOY - 1 + MJD_JAN11901;
    fmjd = ((in.sec / 60.0 + in.min) / 60.0 + in.hour) / 24.0;

    /* mjd -> gps */
    out->week = (uint32_t)((mjd - MJD_JAN61980) / 7.0);
    out->SOW = ((mjd - MJD_JAN61980) - out->week * 7 + fmjd) * SEC_PER_DAY;

    UTC_TIME_2_Modified_julian_date(in, &mjd);
    uint32_t GPS_UTC_DIFF = tai_leap_second(mjd) - TAI_GPS_DIFF;

    double temp = out->SOW + GPS_UTC_DIFF;
    
    if (fabs(GPS_UTC_DIFF) > SEC_PER_WEEK) {
        fprintf(stderr, "Sorry, Can't increment time by >= 1 week\n");
        return 0;
    }

    if (temp < 0.0) {
        out->week = out->week - 1;
        out->SOW = SEC_PER_WEEK + temp;
    } else if (temp >= SEC_PER_WEEK) {
        out->week = out->week + 1;
        out->SOW = temp - SEC_PER_WEEK;
    } else {
        out->SOW = temp;
    }

    return 0;
}

int MJD_2_GPS_TIME(double mjd, GPS_TIME *out) {
    double imjd = floor(mjd);
    double fmjd = mjd - floor(mjd);
    uint32_t GPS_UTC_DIFF = tai_leap_second(mjd) - TAI_GPS_DIFF;

    out->week = (uint32_t)((imjd - MJD_JAN61980) / 7.0);
    out->SOW = ((imjd - MJD_JAN61980) - out->week * 7 + fmjd) * SEC_PER_DAY;

    double temp = out->SOW + GPS_UTC_DIFF;

    if (fabs(GPS_UTC_DIFF) > SEC_PER_WEEK) {
        fprintf(stderr, "Sorry, Can't increment time by >= 1 week\n");
        return 0;
    }

    if (temp < 0.0) {
        out->week = out->week - 1;
        out->SOW = SEC_PER_WEEK + temp;
    } else if (temp >= SEC_PER_WEEK) {
        out->week = out->week + 1;
        out->SOW = temp - SEC_PER_WEEK;
    } else {
        out->SOW = temp;
    }

    return 0;
}

int UTC_TIME_DOY_2_CALENDAR_DATE(uint32_t year_in, uint32_t doy_in, UTC_TIME *out) {
    unsigned int month_array[12] = {31, 28, 31, 30, 31, 30 , 31, 31, 30, 31, 30, 31};
    unsigned int month_array_leap_year[12] = {31, 29, 31, 30, 31, 30 , 31, 31, 30, 31, 30, 31};
    unsigned int month = 0;

    /* check for leap year */
    if ((year_in % 4 == 0 && year_in % 100 != 0) || year_in % 400 == 0) {
        while (doy_in > month_array_leap_year[month]) {
            doy_in = doy_in - month_array_leap_year[month];
            month = month + 1;
        }
    } else {
        while (doy_in > month_array[month]) {
            doy_in = doy_in - month_array[month];
            month = month + 1;
        }
    }

    out->year = year_in;
    out->month = month + 1;
    out->day = doy_in;

    return 0;
}

int UTC_CALENDAR_DATE_2_DOY(UTC_TIME *utc) {
    unsigned int regu_month_day[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unsigned int leap_month_day[12] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    unsigned int yday = 0;

    /* check for leap year */
    if (((utc->year % 4 == 0) && (utc->year % 100 != 0)) || (utc->year % 400 == 0)) {
        yday = leap_month_day[utc->month - 1] + utc->day;
    } else {
        yday = regu_month_day[utc->month - 1] + utc->day;
    }

    utc->DOY = yday;

    return 0;
}

int MJD_2_JD(double mjd, double *jd) {
    *jd = mjd + 2400000.5;
    return 0;
}






