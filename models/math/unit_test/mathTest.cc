#include <climits>
#include "gtest/gtest.h"
#include "math/include/math_utility.hh"

TEST(mathTest, SignFunction) {
    EXPECT_EQ(1, sign(1.0));
    EXPECT_EQ(-1, sign(-1.0));

    EXPECT_EQ(1, sign(DBL_MAX));
    EXPECT_EQ(1, sign(DBL_MIN));
    EXPECT_EQ(-1, sign(-DBL_MAX));
    EXPECT_EQ(-1, sign(-DBL_MIN));

    EXPECT_EQ(-1, sign(-1e-10));
    EXPECT_EQ(1, sign(1e-100));
}
