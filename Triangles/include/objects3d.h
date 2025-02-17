#pragma once

#include "Triangles.h"
#include <concepts>
#include <memory>

using Stereometry::interval_t;
using Stereometry::triangle_t;
using Stereometry::vector_t;

namespace Objects3D {

enum trgle_object_types { POINT, INTERVAL, TRIANGLE };

class obj3d_t {
    trgle_object_types type_;

public:
    obj3d_t(trgle_object_types type) : type_(type) {}
    virtual ~obj3d_t() = default;
    trgle_object_types type() const { return type_; }
    virtual bool is_intersect(const obj3d_t &other) const = 0;
};

template <std::floating_point T> class point_obj_t final : public obj3d_t {
    vector_t<T> p_;

public:
    point_obj_t(const vector_t<T> &p)
        : obj3d_t(trgle_object_types::POINT), p_(p)
    {}
    const vector_t<T> &p() const { return p_; }

    bool is_intersect(const obj3d_t &other) const override;
};

template <std::floating_point T> class interval_obj_t final : public obj3d_t {
    interval_t<T> ival_;

public:
    interval_obj_t(std::pair<const vector_t<T> &, const vector_t<T> &> ends)
        : obj3d_t(trgle_object_types::INTERVAL), ival_(ends)
    {}
    const interval_t<T> &ival() const { return ival_; }

    bool is_intersect(const obj3d_t &other) const override;
};

template <std::floating_point T> class triangle_obj_t final : public obj3d_t {
    triangle_t<T> trgle_;

public:
    triangle_obj_t(const vector_t<T> &p1, const vector_t<T> &p2,
                   const vector_t<T> &p3)
        : obj3d_t(trgle_object_types::TRIANGLE), trgle_(p1, p2, p3)
    {}
    const triangle_t<T> &trgle() const { return trgle_; }

    bool is_intersect(const obj3d_t &other) const override;
};

template <std::floating_point T> class gen_triangle_t final {
    vector_t<T> p1_, p2_, p3_;
    std::unique_ptr<obj3d_t> obj_;

public:
    gen_triangle_t(const vector_t<T> &p1, const vector_t<T> &p2,
                   const vector_t<T> &p3)
        : p1_(p1), p2_(p2), p3_(p3)
    {
        if (p1 == p2 && p2 == p3)
            obj_ = std::make_unique<point_obj_t<T>>(p1);
        else if (Stereometry::are_on_line(p1, p2, p3))
            obj_ = std::make_unique<interval_obj_t<T>>(
                Stereometry::get_farthest(p1, p2, p3));
        else
            obj_ = std::make_unique<triangle_obj_t<T>>(p1, p2, p3);
    }

    trgle_object_types type() const { return obj_->type(); }
    const vector_t<T> &p1() const { return p1_; }
    const vector_t<T> &p2() const { return p2_; }
    const vector_t<T> &p3() const { return p3_; }

    bool is_intersect(const gen_triangle_t<T> &other) const
    {
        return obj_->is_intersect(*other.obj_);
    }
};

} // namespace Objects3D

#include "objects3d.cpp"