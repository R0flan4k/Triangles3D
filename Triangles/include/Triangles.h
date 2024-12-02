#pragma once

#include "double_comparing.h"
#include "matrix.h"

#include <array>
#include <cassert>
#include <cmath>
#include <optional>
#include <utility>

namespace Stereometry {

struct vector_t {
    float x, y, z;

    float len() const;
    void dump() const;
};

bool operator==(const vector_t &lhs, const vector_t &rhs);

// Vector product.
vector_t cross(const vector_t &lhs, const vector_t &rhs);
// Scalar product.
float dot(const vector_t &lhs, const vector_t &rhs);
vector_t operator*(const float coeff, const vector_t &vect);
float operator/(const vector_t &lhs, const vector_t &rhs);
vector_t operator/(const vector_t &lhs, const float coeff);
vector_t operator-(const vector_t &lhs, const vector_t &rhs);
vector_t operator+(const vector_t &lhs, const vector_t &rhs);
bool are_on_line(const vector_t &p1, const vector_t &p2, const vector_t &p3);
bool are_collinear_vect(const vector_t &p1, const vector_t &p2);
std::pair<vector_t, vector_t>
get_farthest(const vector_t &p1, const vector_t &p2, const vector_t &p3);

struct plane_t {
    // Plane equation:
    // ax + by + cz + d = 0.
    float a, b, c, d;

    plane_t(float a, float b, float c, float d);
    plane_t(const vector_t &p1, const vector_t &p2, const vector_t &p3);
    void dump() const;
    bool subset_check(const vector_t &p) const;

    // First pair value is parallelism, second is equality.
    std::pair<bool, bool> is_parallel_equal(const plane_t &pln) const;
    vector_t get_common_point(const plane_t &pln) const;

private:
    vector_t degenerate_get_common_point(const plane_t &pln) const;
};

struct line_t {
    //                _   _    _
    // Line equation: r = r0 + at.
    vector_t r0, a;

    line_t(const plane_t &pln1, const plane_t &pln2);
    line_t(const std::pair<vector_t, vector_t> &p);
    void dump() const;
    bool is_parallel(const plane_t &pln) const;
    bool is_in(const plane_t &pln) const;
    bool subset_check(const vector_t &p) const;
    bool is_intersect(const line_t &line) const;
    float get_distance(const vector_t &line) const;

    // Get coeeficient t of the point on the line of line equation.
    // If the point isn't located on the line returns NAN.
    float get_point_coeff(const vector_t &p) const;

    // Get coeeficient t of the intersection point of this line equation.
    // If these lines don't intersect returns NAN.
    float get_intersection(const line_t &line) const;
    float get_intersection(const plane_t &pln) const;
};

class interval_t {
    line_t l_;

    // Coefficients t in line equation of extreme points
    // of the interval.
    std::pair<float, float> ends_;

public:
    const line_t& line() const {return l_;}
    const std::pair<float, float> ends() const {return ends_;}
    vector_t ends1() const { return l_.r0 + ends_.first * l_.a; }
    vector_t ends2() const { return l_.r0 + ends_.second * l_.a; }

    interval_t(const std::pair<vector_t, vector_t> &ends);
    interval_t(const line_t &l, const std::pair<float, float> ends);
    void dump() const;
    bool subset_check(const vector_t &p) const;
    float len() const;

    // Get coefficient t of intersection of line equation.
    float get_intersection(const line_t &line) const;
    bool is_intersect(const interval_t &ival) const;
    bool is_intersect(const line_t &line) const;

    // Check if the interval intersects another interval,
    // that's located on the same line.
    // If it isn't located on the line, that is UB.
    bool is_intersect_inline(const interval_t &ival) const;
};

class triangle_t {
    vector_t p1_, p2_, p3_;
    plane_t pln_;
    std::array<interval_t, 3> edges_;

public:
    const vector_t &p1() const { return p1_; }
    const vector_t &p2() const { return p2_; }
    const vector_t &p3() const { return p3_; }
    const plane_t& plane() const {return pln_;}
    const std::array<interval_t, 3> &edges() const { return edges_; }
    const interval_t& max_edge() const;

    triangle_t(const vector_t &p1, const vector_t &p2, const vector_t &p3);
    void dump() const;

    // Get coefficients t of intersection points of line equation.
    std::pair<float, float> get_intersection_interval(const line_t &line) const;
    bool is_intersect(const triangle_t &trgle) const;

    // Check if the triangle intersects another triangle,
    // that's located on the same plane.
    // If it isn't located on the plane, that is UB.
    bool is_intersect_inplane(const triangle_t &trgle) const;
    bool is_intersect(const interval_t &ival) const;
    bool is_intersect(const line_t &line) const;
    bool subset_check(const vector_t &p) const;
};

} // namespace Triangles3D