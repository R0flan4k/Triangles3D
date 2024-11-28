#include "Triangles.h"
#include "double_comparing.h"
#include "linear_systems.h"
#include "triangles_exceptions.h"

#include <cassert>
#include <functional>
#include <iostream>
#include <cmath>
#include <algorithm>

using DblCmp::are_eq;
using DblCmp::are_geq;
using DblCmp::is_zero;
using DblCmp::are_intersects_ivals;

bool Stereometry::vector_t::valid() const
{
    return (x.has_value()) && (y.has_value()) && (z.has_value());
}

float Stereometry::vector_t::len() const
{
    return std::sqrt(dot(*this, *this));
}

void Stereometry::vector_t::dump() const
{
    std::cout << *x << ", " << *y << ", " << *z << '.' << std::endl;
}

bool Stereometry::operator==(const vector_t &lhs, const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return (are_eq(*lhs.x, *rhs.x)) && (are_eq(*lhs.y, *rhs.y)) &&
           (are_eq(*lhs.z, *rhs.z));
}

Stereometry::vector_t Stereometry::operator*(const float coeff,
                                             const vector_t &vect)
{
    assert(vect.valid());
    return {coeff * *vect.x, coeff * *vect.y, coeff * *vect.z};
}

Stereometry::vector_t Stereometry::cross(const vector_t &lhs,
                                         const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {*lhs.y * *rhs.z - *lhs.z * *rhs.y,
            *lhs.z * *rhs.x - *lhs.x * *rhs.z,
            *lhs.x * *rhs.y - *lhs.y * *rhs.x};
}

float Stereometry::dot(const vector_t &lhs, const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return *lhs.x * *rhs.x + *lhs.y * *rhs.y + *lhs.z * *rhs.z;
}

Stereometry::vector_t Stereometry::operator/(const vector_t &lhs,
                                             const float coeff)
{
    assert(lhs.valid());
    return {*lhs.x / coeff, *lhs.y / coeff, *lhs.z / coeff};
}

float Stereometry::operator/(const vector_t &lhs, const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    assert(are_collinear_vect(lhs, rhs));
    if (is_zero(*rhs.x))
    {
        if (is_zero(*rhs.y))
            return *lhs.z / *rhs.z;
        return *lhs.y / *rhs.y;
    }
    return *lhs.x / *rhs.x;
}

Stereometry::vector_t Stereometry::operator-(const vector_t &lhs,
                                             const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {*lhs.x - *rhs.x, *lhs.y - *rhs.y, *lhs.z - *rhs.z};
}

Stereometry::vector_t Stereometry::operator+(const vector_t &lhs,
                                             const vector_t &rhs)
{
    assert(lhs.valid() && rhs.valid());
    return {*lhs.x + *rhs.x, *lhs.y + *rhs.y, *lhs.z + *rhs.z};
}

bool Stereometry::are_on_line(const vector_t &p1, const vector_t &p2,
                              const vector_t &p3)
{
    assert(p1.valid() && p2.valid() && p3.valid());
    return are_collinear_vect(p2 - p1, p3 - p2);
}

bool Stereometry::are_collinear_vect(const vector_t &p1, const vector_t &p2)
{
    assert(p1.valid() && p2.valid());
    return (cross(p1, p2)) == vector_t{0, 0, 0};
}

Stereometry::plane_t::plane_t(float aa, float bb, float cc, float dd)
: a(aa), b(bb), c(cc), d(dd) {}

Stereometry::plane_t::plane_t(const vector_t &p1, const vector_t &p2,
                              const vector_t &p3)
{
    assert(p1.valid() && p2.valid() && p3.valid());

    if (are_on_line(p1, p2, p3)) 
        return;
    
    // Vectors that on
    // the plane.
    vector_t v1 = p2 - p1, v2 = p3 - p2;
    // Plane normal vector. n = [v1, v2]
    vector_t n = cross(v1, v2);
#if 0
    // Vector normalization for accurate calculations.
    n = n / n.len();
#endif

    a = *n.x;
    b = *n.y;
    c = *n.z;
    d = -dot(p1, n);
}

bool Stereometry::plane_t::valid() const
{
    return (a.has_value()) && (b.has_value()) && (c.has_value()) &&
           (d.has_value());
}

void Stereometry::plane_t::dump() const
{
    std::cout << "a = " << *a << ", b = " << *b << ", c = " << *c
              << ", d = " << *d << std::endl;
}

bool Stereometry::plane_t::subset_check(const vector_t &p) const
{
    assert(p.valid());
    return is_zero(*p.x * *a + *p.y * *b + *p.z * *c + *d);
}

std::pair<bool, bool> Stereometry::plane_t::is_parallel_equal(const plane_t &pln) const
{
    return {
        (is_zero(*a) ? is_zero(*pln.a) : are_eq(*pln.b, (*pln.a / *a) * *b)) &&
            (is_zero(*b) ? is_zero(*pln.b)
                         : are_eq(*pln.c, (*pln.b / *b) * *c)),
        (is_zero(*c) ? is_zero(*pln.c) : are_eq(*pln.d, (*pln.c / *c) * *d))};
}

static std::pair<float, float>
calculate_linear2d(const Matrices::const_matrix_t<double> &matr, double f1,
                   double f2)
{
    LinearSystems::linear_system_t<double> ls(matr, {f1, f2});
    auto solve_it = ls.calculate_linear();
    return std::make_pair(*(solve_it.first++), *(solve_it.first++));
}

Stereometry::vector_t
Stereometry::plane_t::get_common_point(const plane_t &pln) const
{
    assert(valid() && pln.valid());

    Matrices::const_matrix_t<double> system_matr{*b, *c, *pln.b, *pln.c};
    if (is_zero(system_matr.calculate_det()))
        return degenerate_get_common_point(pln);

    std::pair<float, float> solve;
    float x;
    if (*a == 0)
    {
        if (*pln.a == 0)
        {   
            x = 0;
            solve = calculate_linear2d(system_matr, -*d, -*pln.d);
        }
        else
        {
            x = -*pln.d / *pln.a;
            solve = calculate_linear2d(system_matr, -*d, 0);
        }
    }
    else
    {
        x = -*d / *a;
        solve = calculate_linear2d(system_matr, 0, -*pln.a * x - *pln.d);
    }
    return {x, solve.second, solve.first};
}

Stereometry::vector_t
Stereometry::plane_t::degenerate_get_common_point(const plane_t &pln) const
{
    if (is_zero(*a))
    {
        if (!is_zero(*b))
            return {0, -*d, 0};
        else if (!is_zero(*c))
            return {0, 0, -*d};
        else
        {
            if (is_zero(*d))
                return {0, 0, 0};
            else
                return {};
        }
    }
    // Else
    return {-*d / *a, 0, 0};
}

Stereometry::line_t::line_t(const plane_t &pln1, const plane_t &pln2)
{
    //                        _     _
    // If planes are parallel a and r0 remains invalid.
    if (pln1.is_parallel_equal(pln2).first) return;

    vector_t n1 = {*pln1.a, *pln1.b, *pln1.c}, n2 = {*pln2.a, *pln2.b, *pln2.c};
    a = cross(n1, n2);
#if 0
    a = a / a.len();
#endif
    r0 = pln1.get_common_point(pln2);
}

Stereometry::line_t::line_t(const std::pair<vector_t, vector_t> &p)
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
    vector_t n = {*pln.a, *pln.b, *pln.c};
    return is_zero(dot(a, n));
}

bool Stereometry::line_t::is_in(const plane_t &pln) const
{
    assert(valid() && pln.valid());
    return (is_parallel(pln) && pln.subset_check(r0));
}

bool Stereometry::line_t::subset_check(const vector_t &p) const
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
    vector_t v_mul = cross(a, line.a), diff = r0 - line.r0;
#if 0
    v_mul = v_mul / v_mul.len();
    diff = diff / diff.len();
#endif
    return is_zero(dot(v_mul, diff));
}

float Stereometry::line_t::get_distance(const vector_t &p) const
{
    assert(p.valid());
    vector_t vect = cross((r0 - p), a);
    return vect.len() / a.len();
}

float Stereometry::line_t::get_point_coeff(const vector_t &p) const
{
    assert(valid() && p.valid());
    vector_t diff = p - r0;
    assert(are_collinear_vect(diff, a));
    return diff / a;
}

float Stereometry::line_t::get_intersection(const line_t &line) const
{
    {
        assert(valid() && line.valid());
        assert(is_intersect(line));
        Matrices::const_matrix_t<double> sys_matr{*a.x, -*line.a.x, *a.y,
                                                  -*line.a.y},
            matr1{*line.r0.x - *r0.x, -*line.a.x, *line.r0.y - *r0.y,
                  -*line.a.y};
        if (!is_zero(sys_matr.calculate_det()))
            return matr1.calculate_det() / sys_matr.det();

        Matrices::const_matrix_t<double> alt_sys_matr{*a.y, -*line.a.y, *a.z,
                                                      -*line.a.z},
            alt_matr1{*line.r0.y - *r0.y, -*line.a.y, *line.r0.z - *r0.z,
                      -*line.a.z};
        if (!is_zero(alt_sys_matr.calculate_det()))
            return alt_matr1.calculate_det() / alt_sys_matr.det();

        Matrices::const_matrix_t<double> fin_sys_matr{*a.z, -*line.a.z, *a.x,
                                                      -*line.a.x},
            fin_matr1{*line.r0.z - *r0.z, -*line.a.z, *line.r0.x - *r0.x,
                      -*line.a.x};
        return fin_matr1.calculate_det() / fin_sys_matr.calculate_det();
    }
}

float Stereometry::line_t::get_intersection(const plane_t &pln) const
{
    assert(valid() && pln.valid());

    vector_t n = {*pln.a, *pln.b, *pln.c};
    return -(dot(n, r0) + *pln.d) / (dot(n, a));
}

Stereometry::interval_t::interval_t(const std::pair<vector_t, vector_t> &ends)
    : l_(ends)
{
    if (!l_.valid()) return;
    ends_ = {l_.get_point_coeff(ends.first),
             l_.get_point_coeff(ends.second)};
}

Stereometry::interval_t::interval_t(const line_t &l,
                                    const std::pair<float, float> ends)
    : l_(l), ends_(ends)
{}

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

bool Stereometry::interval_t::subset_check(const vector_t &p) const
{
    assert(p.valid());
    if (!l_.subset_check(p)) return false;
    float coeff = l_.get_point_coeff(p);
    return are_geq((coeff - ends_.first) * (ends_.second - coeff), 0.0f);
}

float Stereometry::interval_t::get_intersection(const line_t &line) const
{
    assert(valid() && line.valid());
    return line.get_intersection(l_);
}

bool Stereometry::interval_t::is_intersect(const line_t &line) const
{
    if (!l_.is_intersect(line))
        return false;
    float ic = l_.get_intersection(line);
    return are_geq((ic - ends_.first) * (ends_.second - ic), 0.0f);
}

bool Stereometry::interval_t::is_intersect(const interval_t &ival) const
{
    return is_intersect(ival.l_) && is_intersect(l_);
}

bool Stereometry::interval_t::is_intersect_inline(const interval_t &ival) const
{
    assert(valid() && ival.valid());
    assert(are_collinear_vect(ival.l_.a, l_.a) &&
           are_collinear_vect(l_.a, ival.l_.r0 - l_.r0));

    return are_intersects_ivals(ends_, ival.ends_);
}

float Stereometry::interval_t::len() const
{
    if (!valid())
        return 0.0f;
    vector_t vect = (ends_.second - ends_.first) * l_.a;
    return vect.len();
}

using EndsT = typename std::pair<Stereometry::vector_t, Stereometry::vector_t>;
Stereometry::triangle_t::triangle_t(const vector_t &p1, const vector_t &p2,
                                    const vector_t &p3)
    : p1_(p1), p2_(p2), p3_(p3), pln_(p1, p2, p3),
      edges_({interval_t{EndsT{p1, p2}}, interval_t{EndsT{p2, p3}},
              interval_t{EndsT{p3, p1}}})
{
}

bool Stereometry::triangle_t::valid() const
{
    return p1_.valid() && p2_.valid() && p3_.valid() &&
           pln_.valid() &&
           edges_[0].valid() && edges_[1].valid() && edges_[2].valid();
}

struct EdgesCompT {
    using edge_t = Stereometry::interval_t;
    bool operator()(const edge_t &lhs, const edge_t &rhs) const
    {
        return lhs.len() < rhs.len();
    }
};

const Stereometry::interval_t& Stereometry::triangle_t::max_edge() const
{
    auto it = std::max_element(edges_.cbegin(), edges_.cend(), EdgesCompT{});
    return *it;
}

bool Stereometry::triangle_t::is_intersect(const line_t &line) const
{
    return line.is_in(pln_) &&
           (edges_[0].is_intersect(line) || edges_[1].is_intersect(line) ||
            edges_[2].is_intersect(line));
}

std::pair<float, float> Stereometry::triangle_t::get_intersection_interval(const line_t &line) const
{
    assert(line.valid());

    bool is_intersect0 = edges_[0].is_intersect(line),
         is_intersect1 = edges_[1].is_intersect(line),
         is_intersect2 = edges_[2].is_intersect(line);

    if (!is_intersect0)
    {
        if (!is_intersect1)
        {
            float inter2 = edges_[2].get_intersection(line);
            return {inter2, inter2};
        }
        else if (!is_intersect2)
        {
            float inter1 = edges_[1].get_intersection(line);
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
            float inter0 = edges_[0].get_intersection(line);
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
    if (!is_intersect(plns_intersection) ||
        !trgle.is_intersect(plns_intersection))
        return false;
    std::pair<float, float> intersection_ival1 = get_intersection_interval(plns_intersection),
                            intersection_ival2 = trgle.get_intersection_interval(plns_intersection);
    return are_intersects_ivals(intersection_ival1, intersection_ival2);
}

bool Stereometry::triangle_t::is_intersect_inplane(const triangle_t &trgle) const
{
    assert(valid() && trgle.valid());
    assert(pln_.is_parallel_equal(trgle.plane()).second);

    return subset_check(trgle.p1_) || subset_check(trgle.p2_) || subset_check(trgle.p3_) ||
           trgle.subset_check(p1_) || trgle.subset_check(p2_) || trgle.subset_check(p3_);
}

bool Stereometry::triangle_t::is_intersect(const interval_t &ival) const
{
    assert(ival.valid());
    assert(valid());

    if (ival.line().is_in(pln_))
        return edges_[0].is_intersect(ival) || edges_[1].is_intersect(ival) ||
               edges_[2].is_intersect(ival);
    else if (ival.line().is_parallel(pln_))
        return false;
    // Else
    vector_t p =
        ival.line().r0 + ival.line().get_intersection(pln_) * ival.line().a;
    return subset_check(p);
}

bool Stereometry::triangle_t::subset_check(const vector_t &p) const
{
    assert(p.valid());
    if (!pln_.subset_check(p)) return false;

    vector_t pa = p1_ - p, pb = p2_ - p, pc = p3_ - p;
    vector_t u = cross(pb, pc), v = cross(pc, pa);
    if (!are_geq(dot(u, v), 0.0f))
        return false;

    vector_t w = cross(pa, pb);
    if (!are_geq(dot(u, w), 0.0f))
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