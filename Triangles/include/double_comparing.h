#pragma once

#include <cmath>
#include <limits>

namespace DblCmp{

template <typename T>
bool is_zero(T num)
{
    return std::abs(num) < std::numeric_limits<T>::epsilon();
}

template <typename T> 
bool are_eq(T num1, T num2)
{
    T diff = num1 - num2;
    num1 = std::abs(num1);
    num2 = std::abs(num2);
    float max = num1 > num2 ? num1 : num2;
    return std::abs(num1 - num2) < max * std::numeric_limits<T>::epsilon();
}

template <typename T>
bool are_geq(T num1, T num2)
{
    return are_eq(num1, num2) || num1 > num2;
}

template <typename T>
bool are_leq(T num1, T num2)
{
    return are_eq(num1, num2) || num1 < num2;
}

}