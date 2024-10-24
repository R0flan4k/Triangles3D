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
    void dump() const;
};
 
bool operator==(const point_t &lhs, const point_t &rhs);

// Vector product.
point_t operator*(const point_t &lhs, const point_t &rhs);
point_t operator*(const float coeff, const point_t &vect);
float operator/(const point_t &lhs, const point_t &rhs);
point_t operator/(const point_t &lhs, const float coeff);
point_t operator-(const point_t &lhs, const point_t &rhs);
point_t operator+(const point_t &lhs, const point_t &rhs);
bool are_on_line(const point_t &p1, const point_t &p2,
                 const point_t &p3);
bool are_collinear_vect(const point_t &p1, const point_t &p2);

struct plane_t {
    // Plane equation:
    // ax + by + cz + d = 0.
    float a, b, c, d; 
                       
    plane_t(float a, float b, float c, float d);
    plane_t(const point_t &p1, const point_t &p2,
            const point_t &p3);
    bool valid() const;
    void dump() const;
    bool subset_check(const point_t &p) const;

    // First pair value is parallelism, second is equality.
    std::pair<bool, bool> is_parallel_equal(const plane_t &pln) const;
    point_t get_common_point(const plane_t &pln) const;
};

struct line_t {
    //                _   _    _
    // Line equation: r = r0 + at.
    point_t r0, a;  

    line_t(const plane_t &pln1, const plane_t &pln2);
    line_t(const std::pair<point_t, point_t> &p);
    bool valid() const;
    void dump() const;
    bool is_parallel(const plane_t &pln) const;
    bool is_in(const plane_t &pln) const;
    bool subset_check(const point_t &p) const;
    bool is_intersect(const line_t &line) const;
    float get_distance(const point_t &line) const;

    // Get coeeficient t of the point on the line of line equation.
    // If the point isn't located on the line returns NAN.
    float get_point_coeff(const point_t &p) const;

    // Get coeeficient t of the intersection point of this line equation.
    // If these lines don't intersect returns NAN.
    float get_intersection(const line_t &line) const;
};

class interval_t {
    line_t l_;

    // Coefficients t in line equation of extreme points
    // of the interval.
    std::pair<float, float> ends_;

public:
    const line_t& line() const {return l_;}
    const std::pair<float, float> ends() const {return ends_;}

    interval_t(const std::pair<point_t, point_t> &ends);
    interval_t(const line_t &l, const std::pair<float, float> ends);
    bool valid() const;
    void dump() const;
    bool subset_check(const point_t &p) const;
    float get_len() const;

    // Get coefficient t of intersection of line equation.
    float get_intersection(const line_t &line) const;
    bool is_intersect(const interval_t &ival) const;

    // Check if the interval intersects another interval,
    // that's located on the same line.
    // If it isn't located on the line, that is UB.
    bool is_intersect_inline(const interval_t &ival) const;
};

class triangle_t {
    point_t p1_, p2_, p3_;
    plane_t pln_;
    std::vector<interval_t> edges_;
    
public:
    const point_t& p1() const {return p1_;}
    const point_t& p2() const {return p2_;}
    const point_t& p3() const {return p3_;}
    const plane_t& plane() const {return pln_;}
    const std::vector<interval_t>& edges() const {return edges_;}
    const interval_t& max_edge() const;
    
    triangle_t(const point_t &p1, const point_t &p2, const point_t &p3);
    bool valid() const;
    void dump() const;

    // Get coefficients t of intersection points of line equation.
    std::pair<float, float> get_intersection_interval(const line_t &line) const;
    bool is_intersect(const triangle_t &trgle) const;

    // Check if the triangle intersects another triangle,
    // that's located on the same plane.
    // If it isn't located on the plane, that is UB.
    bool is_intersect_inplane(const triangle_t &trgle) const;
    bool is_intersect(const interval_t &ival) const;
    bool subset_check(const point_t &p) const;

    // Check if the triangle is degenerated into a interval.
    // Includes degenerate point case.
    bool is_special_interval() const;

    // Check if the triangle is degenerated into a point.
    bool is_special_point() const;

private:
    bool is_intersect_degenerate(const triangle_t &trgle) const;
    bool is_intersect_valid(const triangle_t &trgle) const;
};

} // namespace Triangles3D