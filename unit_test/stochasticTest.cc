#include "gtest/gtest.h"
#include "math/include/stochastic.hh"

TEST(stochasticTest, General) {
    EXPECT_DOUBLE_EQ(1.0, exponential(1.0));
    EXPECT_DOUBLE_EQ(1.0, gauss(0., 1.0));
    double buf = 0.1;
    EXPECT_DOUBLE_EQ(1.0, markov(0., 0., 0., 0., buf));
    EXPECT_DOUBLE_EQ(1.0, rayleigh(0.1));
    EXPECT_DOUBLE_EQ(1.0, uniform(0.1, 0.1));
    EXPECT_DOUBLE_EQ(1.0, unituni());
}
