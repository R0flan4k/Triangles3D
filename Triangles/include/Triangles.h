#pragma once

#include "double_comparing.h"
#include "matrix.h"

#include <cmath>
#include <cassert>
#include <utility>
#include <vector>

namespace Stereometry {

struct point_t {
    float x = NAN, y = NAN, z = NAN;

    bool valid() const;
    float get_len() const;
};
 
bool operator==(const point_t &lhs, const point_t &rhs);
point_t operator*(const point_t &lhs, const point_t &rhs);
point_t operator*(const float coeff, const point_t &vect);
float operator/(const point_t &lhs, const point_t &rhs);
point_t operator-(const point_t &lhs, const point_t &rhs);
point_t operator+(const point_t &lhs, const point_t &rhs);
bool are_on_line(const point_t &p1, const point_t &p2,
                 const point_t &p3);
bool are_collinear_vect(const point_t &p1, const point_t &p2);

struct plane_t {
    float a, b, c, d; // Plane equation:
                       // ax + by + cz + d = 0.
    plane_t(float a, float b, float c, float d);
    plane_t(const point_t &p1, const point_t &p2,
            const point_t &p3);
    bool valid() const;
    bool subset_check(const point_t &p) const;
    std::pair<bool, bool> is_parallel_equal(const plane_t &pln) const;
    point_t get_common_point(const plane_t &pln) const;
};

struct line_t {     //                _   _    _
    point_t r0, a;  // Line equation: r = r0 + at.

    line_t(const plane_t &pln1, const plane_t &pln2);
    line_t(const std::pair<point_t, point_t> &p);
    bool valid() const;
    bool is_parallel(const plane_t &pln) const;
    bool is_in(const plane_t &pln) const;
    bool subset_check(const point_t &p) const;
    bool is_intersect(const line_t &line) const;
    float get_distance(const point_t &line) const;
    float get_point_coeff(const point_t &p) const;
    float get_intersection(const line_t &line) const;
};

class interval_t {
    line_t l_;
    std::pair<float, float> ends_;

public:
    const line_t& line() const {return l_;}
    const std::pair<float, float> ends() const {return ends_;}

    interval_t(const std::pair<point_t, point_t> &ends);
    interval_t(const line_t &l, const std::pair<float, float> ends);
    bool valid() const;
    bool subset_check(const point_t &p) const;
    float get_len() const;
    float get_intersection(const line_t &line) const;
    bool is_intersect(const interval_t &ival) const;
    bool is_intersect_inline(const interval_t &ival) const;
};

class triangle_t {
    point_t p1_, p2_, p3_;
    plane_t pln_;
    std::vector<interval_t> edges_;
    
public:
    const plane_t& plane() const {return pln_;}
    const std::vector<interval_t>& edges() const {return edges_;}
    const interval_t& max_edge() const;
    
    triangle_t(const point_t &p1, const point_t &p2, const point_t &p3);
    bool valid() const;
    std::pair<float, float> get_intersection_interval(const line_t &line) const;
    bool is_intersect(const triangle_t &trgle) const;
    bool is_intersect_inplane(const triangle_t &trgle) const;
    bool is_intersect(const interval_t &ival) const;
    bool subset_check(const point_t &p) const;

private:
    bool is_intersect_degenerate(const triangle_t &trgle) const;
    bool is_intersect_valid(const triangle_t &trgle) const;
    bool is_special_interval() const;
    bool is_special_point() const;
};

// class cube_t {
//     std::pair<point_t, point_t> angles_;

// public:
//     cube_t(std::pair<point_t, point_t> &angles);
//     bool valid() const;
//     bool is_intersect(const cube_t &cube) const;
// };

} // namespace Triangles3D