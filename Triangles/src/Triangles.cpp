// #include "Triangles.h"
#include "double_comparing.h"
#include "linear_systems.h"
#include "triangles_exceptions.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <functional>
#include <iostream>

using DblCmp::are_eq;
using DblCmp::are_geq;
using DblCmp::is_zero;
using DblCmp::are_intersects_ivals;

template <std::floating_point T> T Stereometry::vector_t<T>::len() const
{
    return std::sqrt(dot(*this, *this));
}

template <std::floating_point T> void Stereometry::vector_t<T>::dump() const
{
    std::cout << x << ", " << y << ", " << z << '.' << std::endl;
}

template <std::floating_point T>
bool Stereometry::operator==(const vector_t<T> &lhs, const vector_t<T> &rhs)
{
    return (are_eq(lhs.x, rhs.x)) && (are_eq(lhs.y, rhs.y)) &&
           (are_eq(lhs.z, rhs.z));
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::operator*(const T coeff,
                                                const vector_t<T> &vect)
{
    return {coeff * vect.x, coeff * vect.y, coeff * vect.z};
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::cross(const vector_t<T> &lhs,
                                            const vector_t<T> &rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

template <std::floating_point T>
T Stereometry::dot(const vector_t<T> &lhs, const vector_t<T> &rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::operator/(const vector_t<T> &lhs,
                                                const T coeff)
{
    return {lhs.x / coeff, lhs.y / coeff, lhs.z / coeff};
}

template <std::floating_point T>
T Stereometry::operator/(const vector_t<T> &lhs, const vector_t<T> &rhs)
{
    assert(are_collinear_vect(lhs, rhs));
    if (is_zero(rhs.x))
    {
        if (is_zero(rhs.y))
            return lhs.z / rhs.z;
        return lhs.y / rhs.y;
    }
    return lhs.x / rhs.x;
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::operator-(const vector_t<T> &lhs,
                                                const vector_t<T> &rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::operator+(const vector_t<T> &lhs,
                                                const vector_t<T> &rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

template <std::floating_point T>
bool Stereometry::are_on_line(const vector_t<T> &p1, const vector_t<T> &p2,
                              const vector_t<T> &p3)
{
    return are_collinear_vect(p2 - p1, p3 - p2);
}

template <std::floating_point T>
bool Stereometry::are_collinear_vect(const vector_t<T> &p1,
                                     const vector_t<T> &p2)
{
    return (cross(p1, p2)) == vector_t<T>{0, 0, 0};
}

template <std::floating_point T>
std::pair<Stereometry::vector_t<T>, Stereometry::vector_t<T>>
Stereometry::get_farthest(const vector_t<T> &p1, const vector_t<T> &p2,
                          const vector_t<T> &p3)
{
    vector_t<T> dist1 = p1 - p2, dist2 = p3 - p2, dist3 = p1 - p3;
    size_t len1 = dist1.len(), len2 = dist2.len(), len3 = dist3.len();

    if (len1 > len2)
    {
        if (len1 > len3)
            return std::make_pair(p1, p2);
        else
            return std::make_pair(p1, p3);
    }
    else if (len2 > len3)
        return std::make_pair(p2, p3);
    else
        return std::make_pair(p1, p3);
}

template <std::floating_point T>
Stereometry::plane_t<T>::plane_t(T aa, T bb, T cc, T dd)
    : a(aa), b(bb), c(cc), d(dd)
{}

template <std::floating_point T>
Stereometry::plane_t<T>::plane_t(const vector_t<T> &p1, const vector_t<T> &p2,
                                 const vector_t<T> &p3)
{

    if (are_on_line(p1, p2, p3))
        throw StereoExcepts::plane_error(
            "Trying to construct a plane with points on one straight line.");

    // Vectors that on
    // the plane.
    vector_t<T> v1 = p2 - p1, v2 = p3 - p2;
    // Plane normal vector. n = [v1, v2]
    vector_t<T> n = cross(v1, v2);
#if 0
    // Vector normalization for accurate calculations.
    n = n / n.len();
#endif

    a = n.x;
    b = n.y;
    c = n.z;
    d = -dot(p1, n);
}

template <std::floating_point T> void Stereometry::plane_t<T>::dump() const
{
    std::cout << "a = " << a << ", b = " << b << ", c = " << c << ", d = " << d
              << std::endl;
}

template <std::floating_point T>
bool Stereometry::plane_t<T>::subset_check(const vector_t<T> &p) const
{
    return is_zero(p.x * a + p.y * b + p.z * c + d);
}

template <std::floating_point T>
std::pair<bool, bool>
Stereometry::plane_t<T>::is_parallel_equal(const plane_t<T> &pln) const
{
    return {(is_zero(a) ? is_zero(pln.a) : are_eq(pln.b, (pln.a / a) * b)) &&
                (is_zero(b) ? is_zero(pln.b) : are_eq(pln.c, (pln.b / b) * c)),
            (is_zero(c) ? is_zero(pln.c) : are_eq(pln.d, (pln.c / c) * d))};
}

template <std::floating_point T>
static std::pair<T, T>
calculate_linear2d(const Matrices::const_matrix_t<T> &matr, T f1, T f2)
{
    LinearSystems::linear_system_t<T> ls(matr, {f1, f2});
    auto solve = ls.calculate_linear();
    auto solve_it = solve->cbegin();
    return std::make_pair(*(solve_it++), *(solve_it++));
}

template <std::floating_point T>
Stereometry::vector_t<T>
Stereometry::plane_t<T>::get_common_point(const plane_t<T> &pln) const
{
    Matrices::const_matrix_t<T> system_matr{b, c, pln.b, pln.c};
    if (is_zero(system_matr.calculate_det()))
        return degenerate_get_common_point(pln);

    std::pair<T, T> solve;
    T x;
    if (a == 0)
    {
        if (pln.a == 0)
        {   
            x = 0;
            solve = calculate_linear2d(system_matr, -d, -pln.d);
        }
        else
        {
            x = -pln.d / pln.a;
            solve = calculate_linear2d(system_matr, -d, 0.0f);
        }
    }
    else
    {
        x = -d / a;
        solve = calculate_linear2d(system_matr, 0.0f, -pln.a * x - pln.d);
    }
    return {x, solve.second, solve.first};
}

template <std::floating_point T>
Stereometry::vector_t<T> Stereometry::plane_t<T>::degenerate_get_common_point(
    const plane_t<T> &pln) const
{
    if (is_zero(a))
    {
        if (!is_zero(b))
            return {0, -d, 0};
        else if (!is_zero(c))
            return {0, 0, -d};
        else
        {
            if (is_zero(d))
                return {0, 0, 0};
            else
                return {};
        }
    }
    // Else
    return {-d / a, 0, 0};
}

template <std::floating_point T>
Stereometry::line_t<T>::line_t(const plane_t<T> &pln1, const plane_t<T> &pln2)
{
    //                        _     _
    // If planes are parallel a and r0 remains invalid.
    if (pln1.is_parallel_equal(pln2).first)
        throw StereoExcepts::line_error(
            "Trying to construct a line with two parallel planes.");

    vector_t<T> n1 = {pln1.a, pln1.b, pln1.c}, n2 = {pln2.a, pln2.b, pln2.c};
    a = cross(n1, n2);
#if 0
    a = a / a.len();
#endif
    r0 = pln1.get_common_point(pln2);
}

template <std::floating_point T>
Stereometry::line_t<T>::line_t(const std::pair<vector_t<T>, vector_t<T>> &p)
{
    //                     _     _
    // If points are equal a and r0 remains invalid.
    if (p.first == p.second)
        throw StereoExcepts::line_error(
            "Trying to construct a line with two equal points.");

    a = p.first - p.second;
    r0 = p.first;
}

template <std::floating_point T> void Stereometry::line_t<T>::dump() const
{
    std::cout << "a:  ";
    a.dump();
    std::cout << "r0: ";
    r0.dump();
}

template <std::floating_point T>
bool Stereometry::line_t<T>::is_parallel(const plane_t<T> &pln) const
{
    vector_t<T> n = {pln.a, pln.b, pln.c};
    return is_zero(dot(a, n));
}

template <std::floating_point T>
bool Stereometry::line_t<T>::is_in(const plane_t<T> &pln) const
{
    return (is_parallel(pln) && pln.subset_check(r0));
}

template <std::floating_point T>
bool Stereometry::line_t<T>::subset_check(const vector_t<T> &p) const
{
    return are_collinear_vect(p - r0, a);
}

template <std::floating_point T>
bool Stereometry::line_t<T>::is_intersect(const line_t<T> &line) const
{
    // Check if that line and another line defines one line.
    if (are_collinear_vect(a, line.a) && subset_check(line.r0))
        return true;
    //                           _   _    _     _
    // Two lines intersects if ([a1, a2], r01 - r02) == 0.
    vector_t<T> v_mul = cross(a, line.a), diff = r0 - line.r0;
#if 0
    v_mul = v_mul / v_mul.len();
    diff = diff / diff.len();
#endif
    return is_zero(dot(v_mul, diff));
}

template <std::floating_point T>
T Stereometry::line_t<T>::get_distance(const vector_t<T> &p) const
{
    vector_t<T> vect = cross((r0 - p), a);
    return vect.len() / a.len();
}

template <std::floating_point T>
T Stereometry::line_t<T>::get_point_coeff(const vector_t<T> &p) const
{
    vector_t<T> diff = p - r0;
    assert(are_collinear_vect(diff, a));
    return diff / a;
}

template <std::floating_point T>
T Stereometry::line_t<T>::get_intersection(const line_t<T> &line) const
{
    {
        assert(is_intersect(line));
        Matrices::const_matrix_t<double> sys_matr{a.x, -line.a.x, a.y,
                                                  -line.a.y},
            matr1{line.r0.x - r0.x, -line.a.x, line.r0.y - r0.y, -line.a.y};
        if (!is_zero(sys_matr.calculate_det()))
            return matr1.calculate_det() / sys_matr.det();

        Matrices::const_matrix_t<double> alt_sys_matr{a.y, -line.a.y, a.z,
                                                      -line.a.z},
            alt_matr1{line.r0.y - r0.y, -line.a.y, line.r0.z - r0.z, -line.a.z};
        if (!is_zero(alt_sys_matr.calculate_det()))
            return alt_matr1.calculate_det() / alt_sys_matr.det();

        Matrices::const_matrix_t<double> fin_sys_matr{a.z, -line.a.z, a.x,
                                                      -line.a.x},
            fin_matr1{line.r0.z - r0.z, -line.a.z, line.r0.x - r0.x, -line.a.x};
        return fin_matr1.calculate_det() / fin_sys_matr.calculate_det();
    }
}

template <std::floating_point T>
T Stereometry::line_t<T>::get_intersection(const plane_t<T> &pln) const
{
    vector_t<T> n = {pln.a, pln.b, pln.c};
    return -(dot(n, r0) + pln.d) / (dot(n, a));
}

template <std::floating_point T>
Stereometry::interval_t<T>::interval_t(
    const std::pair<vector_t<T>, vector_t<T>> &ends)
    : l_(ends)
{
    ends_ = {l_.get_point_coeff(ends.first),
             l_.get_point_coeff(ends.second)};
}

template <std::floating_point T>
Stereometry::interval_t<T>::interval_t(const line_t<T> &l,
                                       const std::pair<T, T> ends)
    : l_(l), ends_(ends)
{}

template <std::floating_point T> void Stereometry::interval_t<T>::dump() const
{
    l_.dump();
    std::cout << "Ends: " << ends_.first << " " << ends_.second << std::endl;
}

template <std::floating_point T>
bool Stereometry::interval_t<T>::subset_check(const vector_t<T> &p) const
{
    if (!l_.subset_check(p)) return false;
    T coeff = l_.get_point_coeff(p);
    return are_geq((coeff - ends_.first) * (ends_.second - coeff), 0.0f);
}

template <std::floating_point T>
T Stereometry::interval_t<T>::get_intersection(const line_t<T> &line) const
{
    return line.get_intersection(l_);
}

template <std::floating_point T>
bool Stereometry::interval_t<T>::is_intersect(const line_t<T> &line) const
{
    if (!l_.is_intersect(line))
        return false;
    T ic = l_.get_intersection(line);
    return are_geq((ic - ends_.first) * (ends_.second - ic), 0.0f);
}

template <std::floating_point T>
bool Stereometry::interval_t<T>::is_intersect(const interval_t<T> &ival) const
{
    return is_intersect(ival.l_) && is_intersect(l_);
}

template <std::floating_point T>
bool Stereometry::interval_t<T>::is_intersect_inline(
    const interval_t<T> &ival) const
{
    assert(are_collinear_vect(ival.l_.a, l_.a) &&
           are_collinear_vect(l_.a, ival.l_.r0 - l_.r0));

    return are_intersects_ivals(ends_, ival.ends_);
}

template <std::floating_point T> T Stereometry::interval_t<T>::len() const
{
    vector_t<T> vect = (ends_.second - ends_.first) * l_.a;
    return vect.len();
}

template <std::floating_point T>
using EndsT =
    typename std::pair<Stereometry::vector_t<T>, Stereometry::vector_t<T>>;

template <std::floating_point T>
Stereometry::triangle_t<T>::triangle_t(const vector_t<T> &p1,
                                       const vector_t<T> &p2,
                                       const vector_t<T> &p3)
    : p1_(p1), p2_(p2), p3_(p3), pln_(p1, p2, p3),
      edges_({interval_t<T>{EndsT<T>{p1, p2}}, interval_t<T>{EndsT<T>{p2, p3}},
              interval_t<T>{EndsT<T>{p3, p1}}})
{
}

template <std::floating_point T> struct EdgesCompT {
    using edge_t = Stereometry::interval_t<T>;
    bool operator()(const edge_t &lhs, const edge_t &rhs) const
    {
        return lhs.len() < rhs.len();
    }
};

template <std::floating_point T>
const Stereometry::interval_t<T> &Stereometry::triangle_t<T>::max_edge() const
{
    auto it = std::max_element(edges_.cbegin(), edges_.cend(), EdgesCompT<T>{});
    return *it;
}

template <std::floating_point T>
bool Stereometry::triangle_t<T>::is_intersect(const line_t<T> &line) const
{
    return line.is_in(pln_) &&
           (edges_[0].is_intersect(line) || edges_[1].is_intersect(line) ||
            edges_[2].is_intersect(line));
}

template <std::floating_point T>
std::pair<T, T> Stereometry::triangle_t<T>::get_intersection_interval(
    const line_t<T> &line) const
{
    bool is_intersect0 = edges_[0].is_intersect(line),
         is_intersect1 = edges_[1].is_intersect(line),
         is_intersect2 = edges_[2].is_intersect(line);

    if (!is_intersect0)
    {
        if (!is_intersect1)
        {
            T inter2 = edges_[2].get_intersection(line);
            return {inter2, inter2};
        }
        else if (!is_intersect2)
        {
            T inter1 = edges_[1].get_intersection(line);
            return {inter1, inter1};
        }
        else
            return {edges_[1].get_intersection(line),
                    edges_[2].get_intersection(line)};
    }
    else if (!is_intersect1)
    {
        if (!is_intersect2)
        {
            T inter0 = edges_[0].get_intersection(line);
            return {inter0, inter0};
        }
        else
            return {edges_[0].get_intersection(line),
                    edges_[2].get_intersection(line)};
    }
    else
        return {edges_[0].get_intersection(line),
                edges_[1].get_intersection(line)};
}

template <std::floating_point T>
bool Stereometry::triangle_t<T>::is_intersect(const triangle_t<T> &trgle) const
{
    std::pair<bool, bool> is_parallel_eq = pln_.is_parallel_equal(trgle.plane());
    if (is_parallel_eq.first)
    {
        if (is_parallel_eq.second)
            return is_intersect_inplane(trgle);
        return false;
    }

    line_t<T> plns_intersection{pln_, trgle.plane()};
    if (!is_intersect(plns_intersection) ||
        !trgle.is_intersect(plns_intersection))
        return false;
    std::pair<T, T> intersection_ival1 =
                        get_intersection_interval(plns_intersection),
                    intersection_ival2 =
                        trgle.get_intersection_interval(plns_intersection);
    return are_intersects_ivals(intersection_ival1, intersection_ival2);
}

template <std::floating_point T>
bool Stereometry::triangle_t<T>::is_intersect_inplane(
    const triangle_t<T> &trgle) const
{
    assert(pln_.is_parallel_equal(trgle.plane()).second);

    return subset_check(trgle.p1_) || subset_check(trgle.p2_) || subset_check(trgle.p3_) ||
           trgle.subset_check(p1_) || trgle.subset_check(p2_) || trgle.subset_check(p3_);
}

template <std::floating_point T>
bool Stereometry::triangle_t<T>::is_intersect(const interval_t<T> &ival) const
{
    if (ival.line().is_in(pln_))
        return edges_[0].is_intersect(ival) || edges_[1].is_intersect(ival) ||
               edges_[2].is_intersect(ival);
    else if (ival.line().is_parallel(pln_))
        return false;
    // Else
    vector_t<T> p =
        ival.line().r0 + ival.line().get_intersection(pln_) * ival.line().a;
    return subset_check(p);
}

template <std::floating_point T>
bool Stereometry::triangle_t<T>::subset_check(const vector_t<T> &p) const
{
    if (!pln_.subset_check(p)) return false;

    vector_t<T> pa = p1_ - p, pb = p2_ - p, pc = p3_ - p;
    vector_t<T> u = cross(pb, pc), v = cross(pc, pa);
    if (!are_geq(dot(u, v), 0.0f))
        return false;

    vector_t<T> w = cross(pa, pb);
    if (!are_geq(dot(u, w), 0.0f))
        return false;
    
    return true;
}

template <std::floating_point T> void Stereometry::triangle_t<T>::dump() const
{
    std::cout << "**********************************" << std::endl;
    std::cout << "             Triangle             " << std::endl;
    std::cout << "Points:                           " << std::endl;
    p1_.dump();
    p2_.dump();
    p3_.dump();
    std::cout << "Edges:                            " << std::endl;
    for (size_t i = 0; i < 3; i++)
    {
    std::cout << "[" << i << "]:" << std::endl;
    edges_[i].dump();
    }
    std::cout << "Plane:                            " << std::endl;
    pln_.dump();
    std::cout << "**********************************" << std::endl;
}