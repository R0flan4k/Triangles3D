#include "objects3d.h"

bool Objects3D::point_obj_t::is_intersect(const obj3d_t &other) const
{
    switch (other.type())
    {
    case trgle_object_types::POINT:
        return p_ == static_cast<const point_obj_t &>(other).p();
        break;

    case trgle_object_types::INTERVAL:
        return static_cast<const interval_obj_t &>(other).ival().subset_check(
            p_);
        break;

    case trgle_object_types::TRIANGLE:
        return static_cast<const triangle_obj_t &>(other).trgle().subset_check(
            p_);
        break;

    default:
        assert(0 && "Unreachable.");
        break;
    }
    return false;
}

bool Objects3D::interval_obj_t::is_intersect(const obj3d_t &other) const
{
    switch (other.type())
    {
    case trgle_object_types::POINT:
        return ival_.subset_check(static_cast<const point_obj_t &>(other).p());
        break;

    case trgle_object_types::INTERVAL:
        return ival_.is_intersect(
            static_cast<const interval_obj_t &>(other).ival());
        break;

    case trgle_object_types::TRIANGLE:
        return static_cast<const triangle_obj_t &>(other).trgle().is_intersect(
            ival_);
        break;

    default:
        assert(0 && "Unreachable.");
        break;
    }
    return false;
}

bool Objects3D::triangle_obj_t::is_intersect(const obj3d_t &other) const
{
    switch (other.type())
    {
    case trgle_object_types::POINT:
        return trgle_.subset_check(static_cast<const point_obj_t &>(other).p());
        break;

    case trgle_object_types::INTERVAL:
        return trgle_.is_intersect(
            static_cast<const interval_obj_t &>(other).ival());
        break;

    case trgle_object_types::TRIANGLE:
        return trgle_.is_intersect(
            static_cast<const triangle_obj_t &>(other).trgle());
        break;

    default:
        assert(0 && "Unreachable.");
        break;
    }
    return false;
}