#include "Triangles.h"
#include "matrix.h"
#include "double_comparing.h"

#include <cassert>
#include <functional>
#include <iostream>
#include <cmath>
#include <algorithm>

using DblCmp::are_eq;
using DblCmp::are_geq;
using DblCmp::is_zero;
using DblCmp::are_intersects_ivals;

bool Stereometry::point_t::valid() const
{
    return (x == x) && (y == y) && (z == z);
}

float Stereometry::point_t::get_len() const
{
    return std::sqrt(x * x + y * y + z * z);
}

void Stereometry::point_t::dump() const
{
    std::cout << x << ", " << y << ", " << z << '.' << std::endl;
}

bool Stereometry::operator==(const point_t &lhs, const point_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return (are_eq(lhs.x, rhs.x)) &&
           (are_eq(lhs.y, rhs.y)) &&
           (are_eq(lhs.z, rhs.z));
}

Stereometry::point_t Stereometry::operator*(const float coeff, const point_t &vect)
{
    assert(vect.valid());
    return {coeff * vect.x,
            coeff * vect.y,
            coeff * vect.z};
}

Stereometry::point_t Stereometry::operator*(const point_t &lhs, const point_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {lhs.y * rhs.z - lhs.z * rhs.y,     
            lhs.z * rhs.x - lhs.x * rhs.z, 
            lhs.x * rhs.y - lhs.y * rhs.x};
}

Stereometry::point_t Stereometry::operator/(const point_t &lhs, const float coeff)
{
    assert(lhs.valid());
    return {lhs.x / coeff, lhs.y / coeff, lhs.z / coeff};
}

float Stereometry::operator/(const point_t &lhs, const point_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    assert(are_collinear_vect(lhs, rhs));
    if (is_zero(rhs.x))
    {
        if (is_zero(rhs.y))
            return is_zero(rhs.z) ? NAN : lhs.z / rhs.z;
        return lhs.y / rhs.y;
    }
    return lhs.x / rhs.x;
}

Stereometry::point_t Stereometry::operator-(const point_t &lhs, const point_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z}; 
}

Stereometry::point_t Stereometry::operator+(const point_t &lhs, const point_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z}; 
}

bool Stereometry::are_on_line(const point_t &p1, const point_t &p2,
                              const point_t &p3)
{
    assert(p1.valid() && p2.valid() && p3.valid());
    return are_collinear_vect(p2 - p1, p3 - p2);
}

bool Stereometry::are_collinear_vect(const point_t &p1, const point_t &p2)
{
    assert(p1.valid() && p2.valid());
    return (p1 * p2) == point_t{0, 0, 0};
}

Stereometry::plane_t::plane_t(float aa, float bb, float cc, float dd)
: a(aa), b(bb), c(cc), d(dd) {}

Stereometry::plane_t::plane_t(const point_t &p1, const point_t &p2,
                              const point_t &p3)
{
    assert(p1.valid() && p2.valid() && p3.valid());

    if (are_on_line(p1, p2, p3)) 
    {
        a = b = c = d = NAN;
        return;
    }
    
    // Vectors that on
    // the plane.
    point_t v1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z}; 
    point_t v2 = {p3.x - p2.x, p3.y - p2.y, p3.z - p2.z}; 
    // Plane normal vector. n = [v1, v2]
    point_t n = v1 * v2; 
#if 0
    // Vector normalization for accurate calculations.
    n = n / n.get_len();
#endif

    a = n.x;
    b = n.y;
    c = n.z;
    d = - p1.x * n.x - p1.y * n.y - p1.z * n.z;
}

bool Stereometry::plane_t::valid() const
{
    return (a == a) && (b == b) && 
           (c == c) && (d == d);
}

void Stereometry::plane_t::dump() const
{
    std::cout << "a = " << a << ", b = " << b << ", c = " << c << ", d = " << d << std::endl;
}

bool Stereometry::plane_t::subset_check(const point_t &p) const
{
    assert(p.valid());
    return is_zero(p.x * a + p.y * b + p.z * c + d);
}

std::pair<bool, bool> Stereometry::plane_t::is_parallel_equal(const plane_t &pln) const
{
    return {
        (is_zero(a) ? is_zero(pln.a) : are_eq(pln.b, (pln.a / a) * b)) &&
        (is_zero(b) ? is_zero(pln.b) : are_eq(pln.c, (pln.b / b) * c)),
        (is_zero(c) ? is_zero(pln.c) : are_eq(pln.d, (pln.c / c) * d))
    };
}

Stereometry::point_t Stereometry::plane_t::get_common_point(const plane_t &pln) const
{
    assert(valid() && pln.valid());

    Matrix::matrix2d_t<double> system_matr{b, c,
                                           pln.b, pln.c};
    if (is_zero(system_matr.det())) return degenerate_get_common_point(pln);

    std::pair<float, float> solve;
    float x;
    if (a == 0)
    {
        if (pln.a == 0)
        {   
            x = 0;
            solve = system_matr.calculate_linear(- d, - pln.d);
        }
        else
        {
            x = - pln.d / pln.a;
            solve = system_matr.calculate_linear(- d, 0);
        }
    }
    else
    {
        x = - d / a;
        solve = system_matr.calculate_linear(0, - pln.a * x - pln.d);
    }
    return {x, solve.first, solve.second};
}

Stereometry::point_t Stereometry::plane_t::degenerate_get_common_point(const plane_t &pln) const
{
    if (a == 0)
    {
        if (!is_zero(b))      return {0, -d, 0};
        else if (!is_zero(c)) return {0, 0, -d};
        else
        {
            if (d == 0) return {0, 0, 0};
            else        return {NAN, NAN, NAN};
        }
    }
    // Else
    return {-d / a, 0, 0};
}

Stereometry::line_t::line_t(const plane_t &pln1, const plane_t &pln2)
{
    //                        _     _
    // If planes are parallel a and r0 remains invalid.
    if (pln1.is_parallel_equal(pln2).first) return;

    point_t n1 = {pln1.a, pln1.b, pln1.c},
            n2 = {pln2.a, pln2.b, pln2.c};
    a = n1 * n2;
#if 0
    a = a / a.get_len();
#endif
    r0 = pln1.get_common_point(pln2);
}

Stereometry::line_t::line_t(const std::pair<point_t, point_t> &p)
{
    //                     _     _
    // If points are equal a and r0 remains invalid.
    if (p.first == p.second) return;

    a = p.first - p.second;
    r0 = p.first;
}

bool Stereometry::line_t::valid() const
{
    return r0.valid() && a.valid();
}

void Stereometry::line_t::dump() const
{
    std::cout << "a:  ";
    a.dump();
    std::cout << "r0: ";
    r0.dump();
}

bool Stereometry::line_t::is_parallel(const plane_t &pln) const
{
    return is_zero(a.x * pln.a + a.y * pln.b + a.z * pln.c);
}

bool Stereometry::line_t::is_in(const plane_t &pln) const
{
    assert(valid() && pln.valid());
    return (is_parallel(pln) && pln.subset_check(r0));
}

bool Stereometry::line_t::subset_check(const point_t &p) const
{
    assert(valid() && p.valid());
    return are_collinear_vect(p - r0, a);
}

bool Stereometry::line_t::is_intersect(const line_t &line) const
{
    assert(valid() && line.valid());

    // Check if that line and another line defines one line.
    if (are_collinear_vect(a, line.a) && subset_check(line.r0))
        return true;
    //                           _   _    _     _
    // Two lines intersects if ([a1, a2], r01 - r02) == 0.
    point_t v_mul = a * line.a;
    point_t diff = r0 - line.r0;
#if 0
    v_mul = v_mul / v_mul.get_len();
    diff = diff / diff.get_len();
#endif
    return is_zero(v_mul.x * diff.x + v_mul.y * diff.y + v_mul.z * diff.z);
}

float Stereometry::line_t::get_distance(const point_t &p) const
{
    assert(p.valid());
    point_t vect = (r0 - p) * a;
    return vect.get_len() / a.get_len();
}

float Stereometry::line_t::get_point_coeff(const point_t &p) const
{
    assert(valid() && p.valid());
    point_t diff = p - r0;
    if (!are_collinear_vect(diff, a)) return NAN;
    return diff / a;
}

float Stereometry::line_t::get_intersection(const line_t &line) const
{
    assert(valid() && line.valid());
    
    if (!is_intersect(line)) return NAN;
    Matrix::matrix2d_t<double> sys_matr{a.x, -line.a.x,
                                        a.y, -line.a.y},
                                  matr1{line.r0.x - r0.x, -line.a.x,
                                        line.r0.y - r0.y, -line.a.y};
    
    if (!is_zero(sys_matr.det())) return matr1.det() / sys_matr.det();

    Matrix::matrix2d_t<double> alt_sys_matr{a.y, -line.a.y,
                                            a.z, -line.a.z},
                                  alt_matr1{line.r0.y - r0.y, -line.a.y,
                                            line.r0.z - r0.z, -line.a.z};
    // Returns NAN if sys_matr.det() and alt_sys_matr.det() are 0.
    return alt_matr1.det() / alt_sys_matr.det();
}

Stereometry::interval_t::interval_t(const std::pair<point_t, point_t> &ends)
: l_(ends)
{
    if (!l_.valid()) return;

    ends_ = {l_.get_point_coeff(ends.first),
             l_.get_point_coeff(ends.second)};
}

Stereometry::interval_t::interval_t(const line_t &l, const std::pair<float, float> ends)
: l_(l), ends_(ends) {}

bool Stereometry::interval_t::valid() const
{
    return l_.valid() &&
          (ends_.first == ends_.first) &&
          (ends_.second == ends_.second);
}

void Stereometry::interval_t::dump() const
{
    l_.dump();
    std::cout << "Ends: " << ends_.first << " " << ends_.second << std::endl;
}

bool Stereometry::interval_t::subset_check(const point_t &p) const
{
    assert(p.valid());
    if (!l_.subset_check(p)) return false;
    float coeff = l_.get_point_coeff(p);
    return are_geq((coeff - ends_.first) * (ends_.second - coeff), 0.0f);
}

float Stereometry::interval_t::get_len() const
{
    if (!valid()) return 0.0f;
    point_t vect = (ends_.second - ends_.first) * l_.a;
    return vect.get_len();
}

float Stereometry::interval_t::get_intersection(const line_t &line) const
{
    assert(valid() && line.valid());

    // Coefficient of intersection point (intersection coefficient).
    float ic = l_.get_intersection(line);
    return are_geq((ic - ends_.first) * (ends_.second - ic), 0.0f) ?
           ic : NAN;
}

bool Stereometry::interval_t::is_intersect(const interval_t &ival) const
{
    if (!l_.is_intersect(ival.l_)) return false;

    // Coefficients of intersection point (intersection coefficient).
    float ic1 = get_intersection(ival.l_),     
          ic2 = ival.get_intersection(l_); 
    return are_geq((ic1 - ends_.first) * (ends_.second - ic1), 0.0f) &&
           are_geq((ic2 - ival.ends_.first) * (ival.ends_.second - ic2), 0.0f);
}

bool Stereometry::interval_t::is_intersect_inline(const interval_t &ival) const
{
    assert(valid() && ival.valid());
    assert(are_collinear_vect(ival.l_.a, l_.a) &&
           are_collinear_vect(l_.a, ival.l_.r0 - l_.r0));

    return are_intersects_ivals(ends_, ival.ends_);
}

Stereometry::triangle_t::triangle_t(const point_t &p1, const point_t &p2, const point_t &p3)
: p1_(p1), p2_(p2), p3_(p3), pln_(p1, p2, p3)
{
    using EndsT = typename std::pair<point_t, point_t>;
    edges_.push_back(interval_t{EndsT{p1, p2}});
    edges_.push_back(interval_t{EndsT{p2, p3}});
    edges_.push_back(interval_t{EndsT{p3, p1}});
}

bool Stereometry::triangle_t::valid() const
{
    return p1_.valid() && p2_.valid() && p3_.valid() &&
           pln_.valid() &&
           edges_[0].valid() && edges_[1].valid() && edges_[2].valid();
}

const Stereometry::interval_t& Stereometry::triangle_t::max_edge() const
{
    float len0 = edges_[0].get_len(),
          len1 = edges_[1].get_len(),
          len2 = edges_[2].get_len();
        
    if (len0 > len1)
        return len0 > len2 ? edges_[0] : edges_[2];
    return len1 > len2 ? edges_[1] : edges_[2];
}

std::pair<float, float> Stereometry::triangle_t::get_intersection_interval(const line_t &line) const
{
    assert(line.valid());

    if (!line.is_in(pln_)) return {NAN, NAN};

    float inter1 = edges_[0].get_intersection(line),
          inter2 = edges_[1].get_intersection(line),
          inter3 = edges_[2].get_intersection(line);

    if (std::isnan(inter1)) 
    {
        if (std::isnan(inter2))
            return {inter3, inter3};
        else if (std::isnan(inter3))
            return {inter2, inter2};
        else
            return {inter2, inter3};
    }
    else if (std::isnan(inter2)) 
    {
        if (std::isnan(inter3))
            return {inter1, inter1};
        else 
            return {inter1, inter3};
    }
    else
        return {inter1, inter2};
}

bool Stereometry::triangle_t::is_intersect(const triangle_t &trgle) const
{
    if (is_special_interval() || trgle.is_special_interval())
        return is_intersect_degenerate(trgle);
    return is_intersect_valid(trgle);
}

bool Stereometry::triangle_t::is_intersect_degenerate(const triangle_t &trgle) const
{
    // Check if these triangles are degenerate cases of triangle.
    // In another words if they are intervals or points.
    std::pair<bool, bool> this_ival_p = {is_special_interval(),       
                                         is_special_point()},         
                         trgle_ival_p = {trgle.is_special_interval(), 
                                         trgle.is_special_point()};   

    if (this_ival_p.second)
    {
        if (trgle_ival_p.second)
            return p1_ == trgle.p1_;
        else if (trgle_ival_p.first)
            return trgle.max_edge().subset_check(p1_);
    }
    else if (this_ival_p.first && trgle_ival_p.first)
        return max_edge().is_intersect(trgle.max_edge());
    else if (!this_ival_p.first)
    {
        if (trgle_ival_p.second)
            return subset_check(trgle.p1_);
        else if (trgle_ival_p.first)
            return is_intersect(trgle.max_edge());
    }
    // else
    return trgle.is_intersect_degenerate(*this);
}

bool Stereometry::triangle_t::is_intersect_valid(const triangle_t &trgle) const
{
    assert(valid() && trgle.valid());

    std::pair<bool, bool> is_parallel_eq = pln_.is_parallel_equal(trgle.plane());
    if (is_parallel_eq.first)
    {
        if (is_parallel_eq.second)
            return is_intersect_inplane(trgle);
        return false;
    }

    line_t plns_intersection{pln_, trgle.plane()};
    std::pair<float, float> intersection_ival1 = get_intersection_interval(plns_intersection),
                            intersection_ival2 = trgle.get_intersection_interval(plns_intersection);
    if ((std::isnan(intersection_ival1.first) && std::isnan(intersection_ival1.second)) ||
        (std::isnan(intersection_ival2.first) && std::isnan(intersection_ival2.second)))
        return false;
    interval_t ival1{plns_intersection, intersection_ival1},
               ival2{plns_intersection, intersection_ival2};
    return ival1.is_intersect_inline(ival2);
}

bool Stereometry::triangle_t::is_intersect_inplane(const triangle_t &trgle) const
{
    assert(valid() && trgle.valid());
    assert(pln_.is_parallel_equal(trgle.plane()).second);
    
    for (auto [edges_it1, edges_it2_start] = std::pair{edges_.cbegin(), trgle.edges_.cbegin()}; 
         edges_it1 != edges_.cend(); ++edges_it1, ++edges_it2_start)
        for (auto edges_it2 = edges_it2_start; edges_it2 != trgle.edges_.cend();
             ++edges_it2) 
            if (edges_it1->is_intersect(*edges_it2))
                return true;
    return false;
}

bool Stereometry::triangle_t::is_intersect(const interval_t &ival) const
{
    assert(ival.valid());
    assert(valid());

    return edges_[0].is_intersect(ival) || edges_[1].is_intersect(ival) ||
           edges_[2].is_intersect(ival);
}

bool Stereometry::triangle_t::subset_check(const point_t &p) const
{
    assert(p.valid());
    if (!pln_.subset_check(p)) return false;

    point_t pa = p1_ - p, pb = p2_ - p, pc = p3_ - p;
    point_t u = pb * pc, v = pc * pa;
    if (!are_geq(u.x * v.x + u.y * v.y + u.z * v.z, 0.0f))
        return false;
    
    point_t w = pa * pb;
    if (!are_geq(u.x * w.x + u.y * w.y + u.z * w.z, 0.0f))
        return false;
    
    return true;
}

bool Stereometry::triangle_t::is_special_interval() const
{
    return !pln_.valid();
}

bool Stereometry::triangle_t::is_special_point() const
{
    return !edges_[0].valid();
}

void Stereometry::triangle_t::dump() const
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