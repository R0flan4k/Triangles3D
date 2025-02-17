#include "Triangles.h"
#include "intersections.h"
#include "linear_systems.h"
#include "gtest/gtest.h"

#include <vector>

using namespace Stereometry;

TEST(Stereometry, AreOnLine)
{
    EXPECT_TRUE(are_on_line(vector_t<float>{-1, -1, -1},
                            vector_t<float>{0, 0, 0},
                            vector_t<float>{1, 1, 1}));
    EXPECT_TRUE(are_on_line(vector_t<float>{1024, -1024, 1024},
                            vector_t<float>{-2048, 2048, -2048},
                            vector_t<float>{1024, -1024, 1024}));
    EXPECT_TRUE(are_on_line(vector_t<float>{0, 0, 0}, vector_t<float>{0, 0, 0},
                            vector_t<float>{0, 0, 0}));
    EXPECT_TRUE(are_on_line(vector_t<float>{1, 8, 4}, vector_t<float>{1, 16, 8},
                            vector_t<float>{1, -32, -16}));
    EXPECT_FALSE(are_on_line(vector_t<float>{0, 1, -1},
                             vector_t<float>{0, 0, 0},
                             vector_t<float>{0, -1, -1}));
    EXPECT_FALSE(are_on_line(vector_t<float>{3214, 765, -11414},
                             vector_t<float>{-31, -32145, 4324},
                             vector_t<float>{990, 4, 2213387}));
}

TEST(Plane, DegeneratePlanes)
{
    plane_t<float> pln1{{1, 0, 0}, {-4, 2, 0}, {-1, -1, 0}};

    EXPECT_EQ(pln1.a, 0);
    EXPECT_EQ(pln1.b, 0);
    EXPECT_EQ(pln1.d, 0);
    EXPECT_NE(pln1.c, 0);
}

TEST(Plane, SubsetCheck)
{
    plane_t<float> pln1{{1, 0, 0}, {-4, 2, 0}, {-1, -1, 0}};

    EXPECT_TRUE(pln1.subset_check(vector_t<float>{0, 0, 0}));
    EXPECT_TRUE(pln1.subset_check(vector_t<float>{-41241, 1231, 0}));
}

TEST(Line, DegenerateLines)
{
    plane_t<float> pln1{0, 0, 1, 0}, pln2{1, 0, 0, 0};
    line_t<float> line12{pln1, pln2},
        line3{{vector_t<float>{1, 0, 0}, vector_t<float>{-100, 0, 0}}};

    EXPECT_TRUE(are_collinear_vect(line12.a, vector_t<float>{0, 1, 0}));
    EXPECT_TRUE(are_collinear_vect(line3.a, vector_t<float>{1, 0, 0}));
}

TEST(Line, GetIntersection)
{
    line_t<float> line1{{vector_t<float>{1, 1, 1}, vector_t<float>{2, 2, 2}}},
        line2{{vector_t<float>{-1, 1, 1}, vector_t<float>{-2, 2, 2}}},
        line3{{vector_t<float>{0, 0, 0}, vector_t<float>{1, 1, 1}}},
        line4{{vector_t<float>{2, 2, 2}, vector_t<float>{3, 3, 3}}};

    float inter34 = line3.get_intersection(line4),
          inter12 = line1.get_intersection(line2);
    vector_t<float> pinter12 = line1.r0 + inter12 * line1.a,
                    res12 = vector_t<float>{0, 0, 0};

    EXPECT_EQ(pinter12, res12);
    EXPECT_TRUE(inter34 != inter34);
}

TEST(Triangle, IsIntersect)
{
    gen_triangle_t<float> t1{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}},
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
        t16{{1, 1, 1}, {1, 2, 2}, {0, 0, 0}},
        t17{{1, 2, 3}, {3, 2, 1}, {0, 0, 0}},
        t18{{2, 2, 0}, {2, 2, 3}, {2, 2, 3}},
        t19{{0, 0, -1}, {0, 0, 0}, {0, 0, 1}},
        t20{{0, 1, -1}, {0, 1, 0}, {0, 1, 1}},
        t21{{0, 1, 1}, {0, -1, 1}, {0, 0, -1}},
        t22{{-1, 0, 0}, {0, 0, 0}, {1, 0, 0}},
        t23{{1, 0, 1}, {-1, 0, 1}, {0, 0, -1}},
        t24{{0, -1, 0}, {0, 1, 0}, {0, 1, 0}},
        t25{{0, 0, 0}, {5, 5, 0}, {5, 5, 10}},
        t26{{0.5, 0, 0}, {0, 0, 0}, {0, 0, 0.5}},
        t27{{-2.176, 3.92, 0.8}, {3.824, 0.408, 0.8}, {0, -3.596, 0.8}},
        t28{{-1.088, 1.96, 0.8}, {1.912, 0.204, 0.8}, {0, -1.798, 0.8}};

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
    EXPECT_TRUE(t17.is_intersect(t3));
    EXPECT_TRUE(t3.is_intersect(t17));
    EXPECT_TRUE(t18.is_intersect(t17));
    EXPECT_TRUE(t17.is_intersect(t18));
    EXPECT_TRUE(t19.is_intersect(t2));
    EXPECT_TRUE(t2.is_intersect(t19));
    EXPECT_TRUE(t20.is_intersect(t2));
    EXPECT_TRUE(t2.is_intersect(t20));
    EXPECT_TRUE(t21.is_intersect(t22));
    EXPECT_TRUE(t22.is_intersect(t21));
    EXPECT_TRUE(t23.is_intersect(t24));
    EXPECT_TRUE(t24.is_intersect(t23));
    EXPECT_TRUE(t25.is_intersect(t26));
    EXPECT_TRUE(t26.is_intersect(t25));
    EXPECT_TRUE(t27.is_intersect(t28));
    EXPECT_TRUE(t28.is_intersect(t27));

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
    interval_t<float> i1{{vector_t<float>{0, 0, 0}, vector_t<float>{10, 0, 0}}},
        i2{{vector_t<float>{-1, -1, -1}, vector_t<float>{1, 1, 1}}};

    EXPECT_TRUE(i1.subset_check(vector_t<float>{5, 0, 0}));
    EXPECT_TRUE(i2.subset_check(vector_t<float>{0, 0, 0}));
    EXPECT_FALSE(i2.subset_check(vector_t<float>{2, 2, 2}));
}

TEST(Triangle, SubsetCheck)
{
    triangle_t<float> t1{{0, 0, 0}, {1, 1, 1}, {0, 1, 1}},
        t2{{1, 1, 1}, {1, 2, 2}, {0, 0, 0}};

    EXPECT_TRUE(t1.subset_check(vector_t<float>{0, 0, 0}));
    EXPECT_FALSE(t2.subset_check(vector_t<float>{2, 2, 2}));
}

TEST(TrglesIntersections, OctreeIntersectionsCntr)
{
    std::vector<float> i1{0, 0, 0, 1, 0, 0,   0,    1, 0,   5,   5,  5, 5, 5,
                          5, 5, 5, 5, 0, 0.5, -0.5, 0, 0.5, 0.5, -1, 0, 0},
        i2{1, 1,   0, 3, 1,   0, 1, 3, 0,   0, 0, 0, 1, 0, 0, 0,   1, 0,
           1, 0.5, 0, 1, 0.5, 1, 0, 0, 0.5, 1, 0, 0, 0, 1, 0, 0,   0, 1,
           0, 0,   0, 0, 3,   3, 0, 0, 3,   1, 1, 0, 1, 2, 3, 5,   4, 8,
           9, 9,   9, 9, 9,   9, 9, 9, 9,   8, 8, 8, 8, 8, 8, -10, 8, 8},
        i3{0, 0, 0, 1, 1, 1, 0, 1, 1, 1,   1,   0,    -1, 1,   0,   0,  -1, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 1,   0,   0,    0,  1,   0,   0,  -1, 0,
           0, 0, 0, 1, 1, 1, 0, 1, 1, 0.5, 0.5, 0.5,  1,  1,   1,   0,  1,  1,
           0, 0, 0, 1, 0, 0, 0, 1, 0, 0,   0.5, -0.5, 0,  0.5, 0.5, -1, 0,  0,
           1, 0, 0, 0, 1, 0, 0, 0, 1, 5,   5,   5,    0,  0,   0,   5,  0,  0,
           0, 0, 0, 0, 0, 5, 5, 5, 5, 7,   3,   5,    2,  1,   4,   3,  3,  3},
        i4{1, 1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1,
           1, 2, 2, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
           1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
           2, 2, 2, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 0, 0, 0},
        i5{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 1, 2, 2,
           2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 0, 0, 0},
        i6{1,  2, 3, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           -1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 2, 0, 1, 1},
        i7{1, 2, 3, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0,
           1, 1, 0, 1, 0, 2, 0, 0, 0, 2, 0, 1, 1, 2, 2, 0, 2, 2, 3,  2, 2, 3},
        i8{1,  2, 3, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0,
           -1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 2, 0,  1, 1,
           2,  2, 0, 2, 2, 3, 2, 2, 3, 2, 1, 0, 1, 1, 3, -3, 1, -1},
        i9{1, 0, 0, 0,  1,   0, 0, 0, 1, 0, 0, 0, 5,  5,
           0, 5, 5, 10, 0.5, 0, 0, 0, 0, 0, 0, 0, 0.5};
    TrglesIntersections::octree_trgles_intersect_cntr_t<float> ts1{
        3, 6, i1.cbegin(), i1.cend()},
        ts2{8, 11, i2.cbegin(), i2.cend()}, ts3{12, 10, i3.cbegin(), i3.cend()},
        ts4{8, 3, i4.cbegin(), i4.cend()}, ts5{4, 3, i5.cbegin(), i5.cend()},
        ts6{4, 4, i6.cbegin(), i6.cend()}, ts7{5, 4, i7.cbegin(), i7.cend()},
        ts8{6, 4, i8.cbegin(), i8.cend()}, ts9{3, 11, i9.cbegin(), i9.cend()};
    size_t inters1 = ts1.calculate_intersections(),
           inters2 = ts2.calculate_intersections(),
           inters3 = ts3.calculate_intersections(),
           inters4 = ts4.calculate_intersections(),
           inters5 = ts5.calculate_intersections(),
           inters6 = ts6.calculate_intersections(),
           inters7 = ts7.calculate_intersections(),
           inters8 = ts8.calculate_intersections(),
           inters9 = ts9.calculate_intersections();

    EXPECT_EQ(inters1, 2);
    EXPECT_EQ(inters2, 6);
    EXPECT_EQ(inters3, 12);
    EXPECT_EQ(inters4, 8);
    EXPECT_EQ(inters5, 3);
    EXPECT_EQ(inters6, 2);
    EXPECT_EQ(inters7, 3);
    EXPECT_EQ(inters8, 5);
    EXPECT_EQ(inters9, 3);
}

TEST(LinearSystems, Solving)
{
    LinearSystems::linear_system_t<double> ls({2, 1, 1, 5}, {4, 0});
    auto solve = ls.calculate_linear();
    auto solve_it = solve->cbegin();
    EXPECT_EQ(*(solve_it++), 20.d / 9);
    EXPECT_EQ(*(solve_it++), -4.d / 9);
}