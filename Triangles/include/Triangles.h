#pragma once

#include "double_comparing.h"
#include "matrix.h"

#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <optional>
#include <utility>

namespace Stereometry {

template <std::floating_point T> struct vector_t {
    T x, y, z;

    T len() const;
    void dump() const;
};

template <std::floating_point T>
bool operator==(const vector_t<T> &lhs, const vector_t<T> &rhs);

// Vector product.
template <std::floating_point T>
vector_t<T> cross(const vector_t<T> &lhs, const vector_t<T> &rhs);

// Scalar product.
template <std::floating_point T>
T dot(const vector_t<T> &lhs, const vector_t<T> &rhs);

template <std::floating_point T>
vector_t<T> operator*(const T coeff, const vector_t<T> &vect);

template <std::floating_point T>
T operator/(const vector_t<T> &lhs, const vector_t<T> &rhs);

template <std::floating_point T>
vector_t<T> operator/(const vector_t<T> &lhs, const T coeff);

template <std::floating_point T>
vector_t<T> operator-(const vector_t<T> &lhs, const vector_t<T> &rhs);

template <std::floating_point T>
vector_t<T> operator+(const vector_t<T> &lhs, const vector_t<T> &rhs);

template <std::floating_point T>
bool are_on_line(const vector_t<T> &p1, const vector_t<T> &p2,
                 const vector_t<T> &p3);

template <std::floating_point T>
bool are_collinear_vect(const vector_t<T> &p1, const vector_t<T> &p2);

template <std::floating_point T>
std::pair<vector_t<T>, vector_t<T>> get_farthest(const vector_t<T> &p1,
                                                 const vector_t<T> &p2,
                                                 const vector_t<T> &p3);

template <std::floating_point T> struct plane_t {
    // Plane equation:
    // ax + by + cz + d = 0.
    T a, b, c, d;

    plane_t(T a, T b, T c, T d);
    plane_t(const vector_t<T> &p1, const vector_t<T> &p2,
            const vector_t<T> &p3);
    void dump() const;
    bool subset_check(const vector_t<T> &p) const;

    // First pair value is parallelism, second is equality.
    std::pair<bool, bool> is_parallel_equal(const plane_t<T> &pln) const;
    vector_t<T> get_common_point(const plane_t<T> &pln) const;

private:
    vector_t<T> degenerate_get_common_point(const plane_t<T> &pln) const;
};

template <std::floating_point T> struct line_t {
    //                _   _    _
    // Line equation: r = r0 + at.
    vector_t<T> r0, a;

    line_t(const plane_t<T> &pln1, const plane_t<T> &pln2);
    line_t(const std::pair<vector_t<T>, vector_t<T>> &p);
    void dump() const;
    bool is_parallel(const plane_t<T> &pln) const;
    bool is_in(const plane_t<T> &pln) const;
    bool subset_check(const vector_t<T> &p) const;
    bool is_intersect(const line_t<T> &line) const;
    T get_distance(const vector_t<T> &line) const;

    // Get coeeficient t of the point on the line of line equation.
    // If the point isn't located on the line returns NAN.
    T get_point_coeff(const vector_t<T> &p) const;

    // Get coeeficient t of the intersection point of this line equation.
    // If these lines don't intersect returns NAN.
    T get_intersection(const line_t<T> &line) const;
    T get_intersection(const plane_t<T> &pln) const;
};

template <std::floating_point T> class interval_t {
    line_t<T> l_;

    // Coefficients t in line equation of extreme points
    // of the interval.
    std::pair<T, T> ends_;

public:
    const line_t<T> &line() const { return l_; }
    const std::pair<T, T> ends() const { return ends_; }
    vector_t<T> ends1() const { return l_.r0 + ends_.first * l_.a; }
    vector_t<T> ends2() const { return l_.r0 + ends_.second * l_.a; }

    interval_t(const std::pair<vector_t<T>, vector_t<T>> &ends);
    interval_t(const line_t<T> &l, const std::pair<T, T> ends);
    void dump() const;
    bool subset_check(const vector_t<T> &p) const;
    T len() const;

    // Get coefficient t of intersection of line equation.
    T get_intersection(const line_t<T> &line) const;
    bool is_intersect(const interval_t<T> &ival) const;
    bool is_intersect(const line_t<T> &line) const;

    // Check if the interval intersects another interval,
    // that's located on the same line.
    // If it isn't located on the line, that is UB.
    bool is_intersect_inline(const interval_t<T> &ival) const;
};

template <std::floating_point T> class triangle_t {
    vector_t<T> p1_, p2_, p3_;
    plane_t<T> pln_;
    std::array<interval_t<T>, 3> edges_;

public:
    const vector_t<T> &p1() const { return p1_; }
    const vector_t<T> &p2() const { return p2_; }
    const vector_t<T> &p3() const { return p3_; }
    const plane_t<T> &plane() const { return pln_; }
    const std::array<interval_t<T>, 3> &edges() const { return edges_; }
    const interval_t<T> &max_edge() const;

    triangle_t(const vector_t<T> &p1, const vector_t<T> &p2,
               const vector_t<T> &p3);
    void dump() const;

    // Get coefficients t of intersection points of line equation.
    std::pair<T, T> get_intersection_interval(const line_t<T> &line) const;
    bool is_intersect(const triangle_t<T> &trgle) const;

    // Check if the triangle intersects another triangle,
    // that's located on the same plane.
    // If it isn't located on the plane, that is UB.
    bool is_intersect_inplane(const triangle_t<T> &trgle) const;
    bool is_intersect(const interval_t<T> &ival) const;
    bool is_intersect(const line_t<T> &line) const;
    bool subset_check(const vector_t<T> &p) const;
};

} // namespace Stereometry

#include "Triangles.cpp"