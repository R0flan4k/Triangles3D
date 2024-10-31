#pragma once

#include <cmath>
#include <limits>
#include <concepts>
#include <type_traits>

namespace DblCmp{

template <typename T>
concept floating_point = std::is_floating_point_v<T>;

template <floating_point T>
const T tolerance =  1e+3 * std::numeric_limits<T>::epsilon();

template <floating_point T>
bool is_zero(T num, T epsilon = tolerance<T>)
{
    return std::abs(num) < epsilon;
}

template <floating_point T> 
bool are_eq(T num1, T num2, T epsilon = tolerance<T>)
{
    T diff = std::abs(num1 - num2);
    num1 = std::abs(num1);
    num2 = std::abs(num2);
    T max = num1 > num2 ? num1 : num2;
    return diff < (max + 1) * epsilon; // +1 for num1 == num2 == 0 case.
}

template <floating_point T>
bool are_geq(T num1, T num2, T epsilon = tolerance<T>)
{
    return are_eq(num1, num2, epsilon) || num1 > num2;
}

template <floating_point T>
bool are_leq(T num1, T num2, T epsilon = tolerance<T>)
{
    return are_eq(num1, num2, epsilon) || num1 < num2;
}

template <floating_point T>
bool abs_cmp(T a, T b)
{
    return std::abs(a) < std::abs(b);
}

template <floating_point T>
static bool are_intersects_ivals(std::pair<T, T> i1,
                                 std::pair<T, T> i2)
{
    return are_geq((i2.first  - i1.first) * 
                   (i1.second - i2.first), 0.0f) ||
           are_geq((i1.first  - i2.first) * 
                   (i2.second - i1.first), 0.0f) ||
           are_geq((i1.second - i2.first) *
                   (i2.second - i1.second), 0.0f); 
}

}