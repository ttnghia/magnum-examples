#ifndef Magnum_Examples_FEMSimulationExample_MathDefinitions_h
#define Magnum_Examples_FEMSimulationExample_MathDefinitions_h
/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>
        2020 — Nghia Truong <nghiatruong.vn@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <Magnum/Magnum.h>
#include <vector>

namespace Magnum { namespace Examples {
#ifdef _MSC_VER
#pragma warning (disable: 4127) // condition expression is constant
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': strcpy, strdup, sprintf, vsnprintf, sscanf, fopen
#endif

template<class T> using StdVT = std::vector<T>;

template<int N, class Type> using VecX   = Eigen::Matrix<Type, N, 1>;
template<int N, class Type> using MatXxX = Eigen::Matrix<Type, N, N>;

/* Fixed size vector/matrix */
using Vec2f = VecX<2, Float>;
using Vec3f = VecX<3, Float>;
using Vec4f = VecX<4, Float>;

using Vec2i = VecX<2, int>;
using Vec3i = VecX<3, int>;
using Vec4i = VecX<4, int>;

using Vec2ui = VecX<2, unsigned int>;
using Vec3ui = VecX<3, unsigned int>;
using Vec4ui = VecX<4, unsigned int>;

using Vec2ul = VecX<2, size_t>;
using Vec3ul = VecX<3, size_t>;
using Vec4ul = VecX<4, size_t>;

using Mat2f = MatXxX<2, Float>;
using Mat3f = MatXxX<3, Float>;
using Mat4f = MatXxX<4, Float>;

using Mat3x4f = Eigen::Matrix<Float, 3, 4>;

/* Dynamic size vector/matrix */
using MatX3f = Eigen::Matrix<Float, -1, 3, Eigen::RowMajor>; /* must be row major for memcpy */
using VecXf  = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using MatXf  = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;

/* Sparse matrix */
using DiagonalMatrixf = Eigen::DiagonalMatrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
using SparseMatrixf   = Eigen::SparseMatrix<Float>;
using Tripletf        = Eigen::Triplet<Float, unsigned int>;

/* Access a block of 3 scalars */
#define block3(a) block<3, 1>(3 * (a), 0)
} }
#endif
