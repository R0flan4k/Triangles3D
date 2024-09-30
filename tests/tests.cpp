#include "gtest/gtest.h"
#include "Triangles.h"

using namespace Stereometry;

TEST(Stereometry, AreOnLine)
{
    EXPECT_TRUE(are_on_line({-1, -1, -1}, 
                            {0, 0, 0}, 
                            {1, 1, 1}));
    EXPECT_TRUE(are_on_line({1024, -1024, 1024},
                            {-2048, 2048, -2048},
                            {1024, -1024, 1024}));
    EXPECT_TRUE(are_on_line({0, 0, 0},
                            {0, 0, 0},
                            {0, 0, 0}));
}