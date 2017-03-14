#include "../models/includes/rocket/Euler.hh"
#include "gtest/gtest.h"

#include "math/utility.hh"
#include "math/integrate.hh"
#include "math/matrix/utility.hh"

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
    _Euler_ euler(*(Kinematics*)dum, *(Propulsion*)NULL, *(Forces*)NULL);
    euler.load_angular_velocity(0.1f,0.1f,0.1f);
    EXPECT_EQ(0.1f ,euler.get_ppx());
    EXPECT_EQ(0.1f ,euler.get_qqx());
    EXPECT_EQ(0.1f ,euler.get_rrx());
}
