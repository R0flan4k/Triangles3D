#pragma once

#include <cassert>
#include <utility>
#include <algorithm>

namespace Matrix {

template <typename ElT>
class matrix3d_t {
    ElT matrix_[9] = {};
    ElT det_;

public:
    matrix3d_t(std::initializer_list<ElT> inpt)
    {
        assert(inpt.size() <= std::size(matrix_));
        std::copy(inpt.begin(), inpt.end(), matrix_);

        det_ = matrix_[0] * matrix_[4] * matrix_[8] +
               matrix_[1] * matrix_[5] * matrix_[6] +
               matrix_[3] * matrix_[7] * matrix_[2] -
               matrix_[2] * matrix_[4] * matrix_[6] -
               matrix_[0] * matrix_[5] * matrix_[7] -
               matrix_[1] * matrix_[3] * matrix_[8];
    }

    ElT det() const {return det_;}
};

template <typename ElT>
class matrix2d_t {
    ElT matrix_[4] = {};
    ElT det_;

public:
    matrix2d_t(std::initializer_list<ElT> inpt)
    {
        assert(inpt.size() <= std::size(matrix_));
        std::copy(inpt.begin(), inpt.end(), matrix_);

        det_ = matrix_[0] * matrix_[3] - matrix_[1] * matrix_[2];
    }

    ElT det() const {return det_;}

    std::pair<ElT, ElT> calculate_linear(ElT free_coeff1, ElT free_coeff2) const
    {
        matrix2d_t<ElT> matr1{free_coeff1,  matrix_[1],
                              free_coeff2, matrix_[3]},
                        matr2{matrix_[0], free_coeff1,
                              matrix_[2], free_coeff2};
        return {matr1.det() / det_, matr2.det() / det_};
    }
};

}
