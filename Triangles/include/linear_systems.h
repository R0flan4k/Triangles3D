#pragma once

#include "matrix.h"

#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace LinearSystems {

template <typename T>
class linear_system_t final : public Matrices::matrix_t<T> {
    std::vector<T> free_coeffs_;

public:
    using SolveIt = typename std::vector<T>::const_iterator;

public:
    explicit linear_system_t(std::initializer_list<T> matr,
                             std::initializer_list<T> free)
        : Matrices::matrix_t<T>(matr), free_coeffs_(free)
    {}

    explicit linear_system_t(const Matrices::matrix_t<T> &matr,
                             std::initializer_list<T> free)
        : Matrices::matrix_t<T>(matr), free_coeffs_(free)
    {}

public:
#ifndef NDEBUG
    void dump_internal() const
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
    std::unique_ptr<std::vector<T>> cramers_rule_calc() const
    {
        std::unique_ptr<std::vector<T>> solve =
            std::make_unique<std::vector<T>>();
        double system_matr_det = this->calculate_det();

        if (system_matr_det == T(0))
        {
            for (size_t i = 0; i < this->rank(); ++i)
                solve->push_back(std::numeric_limits<T>::quiet_NaN());
            return solve;
        }

        for (size_t i = 0; i < this->rank(); ++i)
        {
            Matrices::matrix_t<T> tmp(*this);
            tmp.set_col(i, free_coeffs_.cbegin(), free_coeffs_.cend());
            Matrices::matrix_t<T> ctmp{std::move(tmp)};
            solve->push_back(ctmp.calculate_det() / system_matr_det);
        }

        return solve;
    }

public:
    std::unique_ptr<std::vector<T>> calculate_linear() const
    {
        return cramers_rule_calc();
    }
};

} // namespace LinearSystems