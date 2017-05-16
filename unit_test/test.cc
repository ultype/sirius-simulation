#include "gtest/gtest.h"

#include "math/include/time_utility.hh"

namespace time_util {
class GPS_TIME;
class UTC_TIME;
class CCSDS_CUC;
class Modified_julian_date;
}

/*
class Kinematics;
class Propulsion;
class Forces;

class Fake : public Kinematics {
 public:
    arma::mat get_TBI() {
        arma::mat dum;
        return dum;
    }
};

TEST(EulerTest, General) {
    Fake *dum;
    _Euler_ euler(*reinterpret_cast<Kinematics*>dum, *reinterpret_cast<Propulsion*>NULL, *reinterpret_cast<Forces*>NULL);
    euler.load_angular_velocity(0.1f, 0.1f, 0.1f);
    EXPECT_EQ(0.1f, euler.get_ppx());
    EXPECT_EQ(0.1f, euler.get_qqx());
    EXPECT_EQ(0.1f, euler.get_rrx());
}
*/

TEST(timeTest, UTCtoOther) {
    time_util::UTC_TIME utc;

    utc.set_year(2017);
    utc.set_month(5);
    utc.set_day(14);
    utc.set_hour(20);
    utc.set_min(20);
    utc.set_sec(20.0);

    EXPECT_EQ(134, utc.get_day_of_year());

    time_util::GPS_TIME gps(utc);
    time_util::CCSDS_CUC cuc(gps);
    time_util::Modified_julian_date jul(utc);

    EXPECT_EQ(1949, gps.get_week());
    EXPECT_EQ(73220 + 18, gps.get_SOW());

    EXPECT_NEAR(57887.8474537, jul.get_mjd(), 1e-8);
    /* Not yet figured
    EXPECT_EQ(1, cuc.get_C1());
    EXPECT_EQ(1, cuc.get_C2());
    EXPECT_EQ(1, cuc.get_C3());
    EXPECT_EQ(1, cuc.get_C4());
    EXPECT_EQ(1, cuc.get_F1());
    EXPECT_EQ(1, cuc.get_F2());
    */
}
