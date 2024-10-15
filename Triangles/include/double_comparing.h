#pragma once

#include <cmath>
#include <limits>
#include <iostream>

namespace DblCmp{

template <typename T>
const T tolerance =  1000 * std::numeric_limits<T>::epsilon();

template <typename T>
bool is_zero(T num)
{
    return std::abs(num) < tolerance<T>;
}

template <typename T> 
bool are_eq(T num1, T num2)
{
    float diff = std::abs(num1 - num2);
    num1 = std::abs(num1);
    num2 = std::abs(num2);
    float max = num1 > num2 ? num1 : num2;
    return diff < (max + 1) * tolerance<T>; // +1 for num1 == num2 == 0 case.
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