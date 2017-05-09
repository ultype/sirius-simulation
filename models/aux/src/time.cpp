#include "Time_management.hh"

time_management::time_management() {
    last_time = 0;
}

void time_management::dm_time() { /* convert simulation time to gps time */
    double this_time = get_rettime();

    /* Get current GPS time */
    /* deduct DM_dt because PV comes from dm_att() output of previous cycle */
    gpstime += this_time - last_time;

    last_time = this_time;
}

// Load simulation start time in Calender date form and UTC time and convert into gps time form
void time_management::load_start_time(unsigned int Year, unsigned int DOY, unsigned int Hour, unsigned int Min, unsigned int Sec) {
    time_util::UTC_TIME caldate;

    caldate.set_day_of_year(Year, DOY);
    caldate.set_hour(Hour);
    caldate.set_min(Min);
    caldate.set_sec(Sec);

    this->gpstime = time_util::GPS_TIME(caldate);
}

time_util::GPS_TIME time_management::get_gpstime() {
    // cout<<setprecision(10)<<"gps week: "<<gpstime.get_week()<<'\t'<<"gps sow :"<<gpstime.get_SOW()<<'\t'<<endl;
    return gpstime;
}

time_util::UTC_TIME time_management::get_utctime() {
    time_util::UTC_TIME ret(gpstime);
    // cout<<setprecision(10)<<"Year: "<<ret.get_year()<<'\t'<<"Month: "<<ret.get_month()<<'\t'<<"Day: "<<ret.get_day()<<'\t'<<"Min: "<<ret.get_min()<<'\t'<<"Sec: "<<ret.get_sec()<<endl;
    return ret;
}

time_util::Modified_julian_date time_management::get_modified_julian_date() {
    time_util::Modified_julian_date ret(gpstime);
    // cout<<setprecision(20)<<"Julian_Date: "<<ret.get_jd()<<endl;
    return ret;
}
