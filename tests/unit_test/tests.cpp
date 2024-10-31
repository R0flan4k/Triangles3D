#include "gtest/gtest.h"
#include "Triangles.h"
#include "intersections.h"

#include <vector>

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
               t3{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
               t4{{1, 0, 0}, {0, 1, 0}, {0, -1, 0}},
               t5{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}},
               t6{{0.5, 0.5, 0.5}, {1, 1, 1}, {0, 1, 1}},
               t7{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
               t8{{0, 0.5, -0.5}, {0, 0.5, 0.5}, {-1, 0, 0}},
               t9{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
               t10{{5, 5, 5}, {0, 0, 0}, {5, 0, 0}},
               t11{{0, 0, 0}, {0, 0, 5}, {5, 5, 5}},
               t12{{7, 3, 5}, {2, 1, 4}, {3, 3, 3}},
               t13{{1, 1, 1}, {1, 2, 2}, {2, 2, 2}},
               t14{{1, 1, 1}, {1, 1, 1}, {1, 1, 1}},
               t15{{2, 2, 2}, {2, 2, 2}, {2, 2, 2}},
               t16{{1, 1, 1}, {1, 2, 2}, {0, 0, 0}};
    
    EXPECT_FALSE(t6.is_intersect(t2));
    EXPECT_FALSE(t2.is_intersect(t6));
    EXPECT_FALSE(t15.is_intersect(t16));
    EXPECT_FALSE(t16.is_intersect(t15));
    EXPECT_TRUE(t1.is_intersect(t2));
    EXPECT_TRUE(t2.is_intersect(t1));
    EXPECT_TRUE(t2.is_intersect(t3));
    EXPECT_TRUE(t3.is_intersect(t2));
    EXPECT_TRUE(t1.is_intersect(t3));
    EXPECT_TRUE(t3.is_intersect(t1));
    EXPECT_TRUE(t4.is_intersect(t5));
    EXPECT_TRUE(t5.is_intersect(t4));
    EXPECT_TRUE(t7.is_intersect(t8));
    EXPECT_TRUE(t8.is_intersect(t7));
    EXPECT_TRUE(t9.is_intersect(t10));
    EXPECT_TRUE(t10.is_intersect(t9));
    EXPECT_TRUE(t11.is_intersect(t12));
    EXPECT_TRUE(t12.is_intersect(t11));
    EXPECT_TRUE(t13.is_intersect(t14));
    EXPECT_TRUE(t14.is_intersect(t13));

    EXPECT_TRUE(t1.is_intersect(t1));
    EXPECT_TRUE(t2.is_intersect(t2));
    EXPECT_TRUE(t3.is_intersect(t3));
    EXPECT_TRUE(t4.is_intersect(t4));
    EXPECT_TRUE(t5.is_intersect(t5));
    EXPECT_TRUE(t6.is_intersect(t6));
    EXPECT_TRUE(t7.is_intersect(t7));
    EXPECT_TRUE(t8.is_intersect(t8));
    EXPECT_TRUE(t9.is_intersect(t9));
    EXPECT_TRUE(t10.is_intersect(t10));
    EXPECT_TRUE(t11.is_intersect(t11));
    EXPECT_TRUE(t12.is_intersect(t12));
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
    triangle_t t1{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}},
               t2{{1, 1, 1}, {1, 2, 2}, {0, 0, 0}};
    
    EXPECT_TRUE(t1.subset_check(point_t{0, 0, 0}));
    EXPECT_FALSE(t2.subset_check(point_t{2, 2, 2}));
}

TEST(TrglesIntersections, OctreeIntersectionsCntr)
{
    std::vector<float> i1{0, 0, 0, 1, 0, 0, 0, 1, 0,
                          5, 5, 5, 5, 5, 5, 5, 5, 5,
                          0, 0.5, -0.5, 0, 0.5, 0.5, -1, 0, 0},
                       i2{1, 1, 0, 3, 1, 0, 1, 3, 0,
                          0, 0, 0, 1, 0, 0, 0, 1, 0,
                          1, 0.5, 0, 1, 0.5, 1, 0, 0, 0.5,
                          1, 0, 0, 0, 1, 0, 0, 0, 1,
                          0, 0, 0, 0, 3, 3, 0, 0, 3,
                          1, 1, 0, 1, 2, 3, 5, 4, 8,
                          9, 9, 9, 9, 9, 9, 9, 9, 9,
                          8, 8, 8, 8, 8, 8, -10, 8, 8},
                       i3{0, 0, 0, 1, 1, 1, 0, 1, 1,
                          1, 1, 0, -1, 1, 0, 0, -1, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0,
                          1, 0, 0, 0, 1, 0, 0, -1, 0,
                          0, 0, 0, 1, 1, 1, 0, 1, 1,
                          0.5, 0.5, 0.5, 1, 1, 1, 0, 1, 1,
                          0, 0, 0, 1, 0, 0, 0, 1, 0,
                          0, 0.5, -0.5, 0, 0.5, 0.5, -1, 0, 0,
                          1, 0, 0, 0, 1, 0, 0, 0, 1,
                          5, 5, 5, 0, 0, 0, 5, 0, 0,
                          0, 0, 0, 0, 0, 5, 5, 5, 5,
                          7, 3, 5, 2, 1, 4, 3, 3, 3},
                       i4{1, 1, 1, 1, 2, 2, 2, 2, 2,
                          1, 1, 1, 1, 1, 1, 1, 1, 1,
                          1, 2, 2, 1, 2, 2, 1, 2, 2,
                          2, 2, 2, 2, 2, 2, 2, 2, 2,
                          1, 1, 1, 1, 2, 2, 1, 1, 1,
                          1, 1, 1, 2, 2, 2, 2, 2, 2,
                          2, 2, 2, 1, 2, 2, 2, 2, 2,
                          1, 1, 1, 1, 2, 2, 0, 0, 0},
                       i5{1, 1, 1, 1, 1, 1, 1, 1, 1,
                          1, 2, 2, 1, 2, 2, 1, 2, 2,
                          2, 2, 2, 2, 2, 2, 2, 2, 2,
                          1, 1, 1, 1, 2, 2, 0, 0, 0};
    TrglesIntersections::octree_trgles_intersect_cntr_t 
        ts1{3,  6,  i1.cbegin(), i1.cend()},
        ts2{8,  11, i2.cbegin(), i2.cend()},
        ts3{12, 10, i3.cbegin(), i3.cend()},
        ts4{8,  3,  i4.cbegin(), i4.cend()},
        ts5{4,  3,  i5.cbegin(), i5.cend()};
    size_t inters1 = ts1.calculate_intersections(),
           inters2 = ts2.calculate_intersections(),
           inters3 = ts3.calculate_intersections(),
           inters4 = ts4.calculate_intersections(),
           inters5 = ts5.calculate_intersections();

    EXPECT_EQ(inters1, 2);
    EXPECT_EQ(inters2, 5);
    EXPECT_EQ(inters3, 12);
    EXPECT_EQ(inters4, 8);
    EXPECT_EQ(inters5, 3);
}