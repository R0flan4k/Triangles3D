#pragma once

#include <cmath>

namespace DblCmp{

const float FLT_TOLERANCE = 1e-5;

template <typename T> 
bool are_eq(T num1, T num2)
{
    return std::abs(num1 - num2) < FLT_TOLERANCE;
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