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

TEST(Stereometry, VectorProduct)
{
    point_t vect1 = point_t{1, 0, 0} * point_t{0, 1, 0},
            res1  = point_t{0, 0, 1},
            vect2 = point_t{-5, 2, 0} * point_t{3, -3, 0},
            res2  = point_t{0, 0, 9},
            vect3 = point_t{0, 0, 0} * point_t{0, 0, 0},
            res3  = point_t{0, 0, 0},
            vect4 = point_t{1521, -414, 51} * point_t{738, -31, 859},
            res4  = point_t{-354045, -1268901, 258381};

    EXPECT_EQ(vect1, res1);
    EXPECT_EQ(vect2, res2);
    EXPECT_EQ(vect3, res3);
    EXPECT_EQ(vect4, res4);
}

TEST(Plane, DegeneratePlanes)
{
    plane_t pln1{{1, 0, 0}, {-4, 2, 0}, {-1, -1, 0}},
            pln2{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
            pln3{{0, 0, 0}, {0, 0, 0}, {1, 2, 3}};

    EXPECT_EQ(pln1.a, 0);
    EXPECT_EQ(pln1.b, 0);
    EXPECT_EQ(pln1.d, 0);
    EXPECT_NE(pln1.c, 0);
    EXPECT_FALSE(pln2.valid());
    EXPECT_FALSE(pln3.valid());
}

TEST(Plane, SubsetCheck)
{
    plane_t pln1{{1, 0, 0}, {-4, 2, 0}, {-1, -1, 0}};  

    EXPECT_TRUE(pln1.subset_check(point_t{0, 0, 0}));
    EXPECT_TRUE(pln1.subset_check(point_t{-41241, 1231, 0}));
}

TEST(Line, DegenerateLines)
{
    plane_t pln1{0, 0, 1, 0},
            pln2{1, 0, 0, 0};
    line_t line12{pln1, pln2},
           line3{{point_t{1, 0, 0}, point_t{-100, 0, 0}}};

    EXPECT_TRUE(are_collinear_vect(line12.a, point_t{0, 1, 0}));
    EXPECT_TRUE(are_collinear_vect(line3.a, point_t{1, 0, 0}));
}

TEST(Line, GetIntersection)
{
    line_t line1{{point_t{1, 1, 1}, point_t{2, 2, 2}}},
           line2{{point_t{-1, 1, 1}, point_t{-2, 2, 2}}},
           line3{{point_t{0, 0, 0}, point_t{1, 1, 1}}},
           line4{{point_t{2, 2, 2}, point_t{3, 3, 3}}};

    float inter34 = line3.get_intersection(line4),
          inter12 = line1.get_intersection(line2);
    point_t pinter12 = line1.r0 + inter12 * line1.a,
            res12 = point_t{0, 0, 0};

    EXPECT_EQ(pinter12, res12);
    EXPECT_TRUE(inter34 != inter34);
}

TEST(Triangle, IsIntersect)
{
    triangle_t t1{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}},
               t2{{1, 1, 0}, {-1, 1, 0}, {0, -1, 0}},
               t3{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    
    EXPECT_TRUE(t1.is_intersect(t2));
    EXPECT_TRUE(t2.is_intersect(t3));
    EXPECT_TRUE(t1.is_intersect(t3));
    EXPECT_TRUE(t1.is_intersect(t1));
    EXPECT_TRUE(t2.is_intersect(t2));
    EXPECT_TRUE(t3.is_intersect(t3));
}

TEST(Interval, SubsetCheck)
{
    interval_t i1{{point_t{0, 0, 0}, point_t{10, 0, 0}}},
               i2{{point_t{-1, -1, -1}, point_t{1, 1, 1}}};
    
    EXPECT_TRUE(i1.subset_check(point_t{5, 0, 0}));
    EXPECT_TRUE(i2.subset_check(point_t{0, 0, 0}));
    EXPECT_FALSE(i2.subset_check(point_t{2, 2, 2}));
}

TEST(Triangle, SubsetCheck)
{
    triangle_t t1{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}};
    
    EXPECT_TRUE(t1.subset_check(point_t{0, 0, 0}));
}