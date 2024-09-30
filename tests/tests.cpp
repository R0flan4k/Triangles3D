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
    EXPECT_TRUE(are_on_line({1, 8, 4},
                            {1, 16, 8},
                            {1, -32, -16}));
    EXPECT_FALSE(are_on_line({0, 1, -1},
                             {0, 0, 0},
                             {0, -1, -1}));
    EXPECT_FALSE(are_on_line({3214, 765, -11414},
                             {-31, -32145, 4324},
                             {990, 4, 2213387}));
}

