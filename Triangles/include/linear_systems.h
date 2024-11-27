#pragma once

#include "matrix.h"

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace LinearSystems {

template <typename T>
class linear_system_t final : public Matrices::const_matrix_t<T> {
    std::vector<T> free_coeffs_;
    std::vector<T> solve_;

public:
    using SolveIt = typename std::vector<T>::const_iterator;

public:
    std::pair<SolveIt, SolveIt> solve() const
    {
        return std::make_pair(solve_.cbeing(), solve_.cend());
    }

    explicit linear_system_t(std::initializer_list<T> matr,
                             std::initializer_list<T> free)
        : Matrices::const_matrix_t<T>(matr), free_coeffs_(free)
    {}

    explicit linear_system_t(const Matrices::const_matrix_t<T> &matr,
                             std::initializer_list<T> free)
        : Matrices::const_matrix_t<T>(matr), free_coeffs_(free)
    {}

public:
#ifndef NDEBUG
    void dump_internal() const override
    {
        for (std::size_t i = 0, r = this->rank(); i < r; ++i)
        {
            for (std::size_t j = 0; j < r; ++j)
                std::cout << *this->access(i, j) << "\t";
            std::cout << " | " << free_coeffs_[i] << std::endl;
        }
    }
#endif // NDEBUG

private:
    void cramers_rule_calc()
    {
        if (this->det() == T(0))
        {
            for (size_t i = 0; i < this->rank(); ++i)
                solve_.push_back(std::numeric_limits<T>::quiet_NaN());
            return;
        }

        for (size_t i = 0; i < this->rank(); ++i)
        {
            Matrices::matrix_t<T> tmp(*this);
            tmp.set_col(i, free_coeffs_.cbegin(), free_coeffs_.cend());
            Matrices::const_matrix_t<T> ctmp{std::move(tmp)};
            solve_.push_back(ctmp.calculate_det() / this->det());
        }
    }

public:
    std::pair<SolveIt, SolveIt> calculate_linear()
    {
        if (!solve_.empty() || this->rank() == 0)
            return std::make_pair(solve_.cbegin(), solve_.cend());
        this->calculate_det();
        cramers_rule_calc();
        return std::make_pair(solve_.cbegin(), solve_.cend());
    }
};

} // namespace LinearSystems