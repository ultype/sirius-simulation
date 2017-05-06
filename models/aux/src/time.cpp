#include "Time_management.hh"

time_management::time_management() {
    last_time = 0;

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

void time_management::dm_time() { /* convert simulation time to gps time */
    double this_time = get_rettime();

    /* Get current GPS time */
    /* deduct DM_dt because PV comes from dm_att() output of previous cycle */
    gps_increment(&utctime, this_time - last_time);

    last_time = this_time;

    utctime2gpstime(&utctime, &gpstime);

    /* Convert UTC time to MJD & JD time */
    Julian_Date = gps_to_mjd(&utctime) + 2400000.5;  /* DM_Julian_Date : Julian Date */
    gps_to_caldate(&utctime, &caldate);  // Convert UTC(GPS form) time to calendar Date
    // gps_to_utc(&gpstime, &caldate);
    // cout<<setprecision(10)<<"gps week: "<<gpstime.Week<<'\t'<<"gps sow :"<<gpstime.SOW<<'\t'<<"utc week:"<<utctime.Week<<'\t'<<"utc sow :"<<utctime.SOW<<endl;
    // cout<<setprecision(10)<<"Year: "<<caldate.Year<<'\t'<<"Month: "<<caldate.Month<<'\t'<<"Day: "<<caldate.Day<<'\t'<<"Min: "<<caldate.Min<<'\t'<<"Sec: "<<caldate.Sec<<endl;
    // cout<<setprecision(20)<<"Julian_Date: "<<Julian_Date<<endl;
}

// Load simulation start time in Calender date form and UTC time and convert into gps time form
void time_management::load_start_time(unsigned int Year, unsigned int DOY, unsigned int Hour, unsigned int Min, unsigned int Sec) {
    this->caldate.Year = Year;
    this->caldate.DOY = DOY;
    this->caldate.Hour = Hour;
    this->caldate.Min = Min;
    this->caldate.Sec = Sec;

    caldoy_to_gps(&caldate, &utctime);  // convert into gps time
    utctime2gpstime(&utctime, &gpstime);
    /* Convert UTC time to MJD & JD time */
    Julian_Date = gps_to_mjd(&utctime) + 2400000.5; /* DM_Julian_Date : Julian Date */
}

time_util::GPS_TIME_t time_management::get_gpstime() {
    return gpstime;
}

time_util::GPS_TIME_t time_management::get_utctime() {
    return utctime;
}

time_util::CAL_DATE_t time_management::get_caldate() {
    return caldate;
}

double time_management::get_julian_date() {
    return Julian_Date;
}
