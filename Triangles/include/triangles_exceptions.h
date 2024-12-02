#pragma once

#include <stdexcept>

namespace StereoExcepts {

class stereometry_error : public std::runtime_error {
public:
    stereometry_error(const std::string &what_arg)
        : std::runtime_error(what_arg)
    {}
    stereometry_error(const stereometry_error &) = default;
};

class plane_error : public stereometry_error {
public:
    plane_error(const std::string &what_arg) : stereometry_error(what_arg) {}
    plane_error(const plane_error &) = default;
};

class line_error : public stereometry_error {
public:
    line_error(const std::string &what_arg) : stereometry_error(what_arg) {}
    line_error(const line_error &) = default;
};

} // namespace StereoExcepts