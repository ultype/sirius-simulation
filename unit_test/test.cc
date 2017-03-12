#include "../models/includes/rocket/Euler.hh"
#include "gtest/gtest.h"

class Kinematics;
class Propulsion;
class Forces;

TEST(EulerTest, General) {
    _Euler_ euler(*(Kinematics*)NULL, *(Propulsion*)NULL, *(Forces*)NULL);
    euler.load_angular_velocity(0.1f,0.1f,0.1f);
    EXPECT_EQ(0.1f ,euler.get_ppx());
    EXPECT_EQ(0.1f ,euler.get_qqx());
    EXPECT_EQ(0.1f ,euler.get_rrx());
}
