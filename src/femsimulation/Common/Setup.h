/**
 * Copyright 2019 Nghia Truong <nghiatruong.vn@gmail.com>
 *                Kui Wu <walker.kui.wu@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/****************************************************************************************************/
/* Defaultr floating point number type */
using real = float;

/****************************************************************************************************/
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define SAFETY_CHECK 1

/* Fixed size vector/matrix */
template<int N, class Type> using VecX   = Eigen::Matrix<Type, N, 1>;
template<int N, class Type> using MatXxX = Eigen::Matrix<Type, N, N>;

template<class Type> using Vec2   = Eigen::Matrix<Type, 2, 1>;
template<class Type> using Vec3   = Eigen::Matrix<Type, 3, 1>;
template<class Type> using Mat2x2 = Eigen::Matrix<Type, 2, 2>;
template<class Type> using Mat3x3 = Eigen::Matrix<Type, 3, 3>;

/* Dynamic size vector/matrix */
template<class Type> using DVecX   = Eigen::Matrix<Type, Eigen::Dynamic, 1>;
template<class Type> using DMatXxX = Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>;

/* Common types */
using Vec2i = VecX<2, int>;
using Vec3i = VecX<3, int>;
using Vec4i = VecX<4, int>;

using Vec2ui = VecX<2, unsigned int>;
using Vec3ui = VecX<3, unsigned int>;
using Vec4ui = VecX<4, unsigned int>;

using Vec2ul = VecX<2, size_t>;
using Vec3ul = VecX<3, size_t>;
using Vec4ul = VecX<4, size_t>;

using Vec2r = VecX<2, real>;
using Vec3r = VecX<3, real>;
using Vec4r = VecX<4, real>;

using Mat2r = MatXxX<2, real>;
using Mat3r = MatXxX<3, real>;
using Mat4r = MatXxX<4, real>;

using VecXr = DVecX<real>;
using MatXr = DMatXxX<real>;

using SparseVector        = Eigen::SparseVector<real>;
using SparseMatrix        = Eigen::SparseMatrix<real>;
using SparseMatrixTriplet = Eigen::Triplet<real, int>;

/****************************************************************************************************/
#include <string>
#include <cstdint>
#include <vector>

using String = std::string;

using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;
using f64 = double;

template<class T> using StdVT = std::vector<T>;

/****************************************************************************************************/
#define PRINT_LINE                              \
    {                                           \
        printf("%d: %s\n", __LINE__, __FILE__); \
        fflush(stdout);                         \
    }

#define PRINT_LOCATION                                        \
    {                                                         \
        std::stringstream ss;                                 \
        ss << "Function: " << __func__ << std::endl;          \
        ss << "Line: " << __LINE__ << ", file: " << __FILE__; \
        printf("%s\n", ss.str().c_str());                     \
        fflush(stdout);                                       \
    }

#define REQUIRE(condition)                                                    \
    {                                                                         \
        if(!(condition))                                                      \
        {                                                                     \
            String erMsg = String("Assertion failed: ") + String(#condition); \
            printf("%s\n", erMsg.c_str());                                    \
            PRINT_LOCATION                                                    \
            exit(EXIT_FAILURE);                                               \
        }                                                                     \
    }

/****************************************************************************************************/
#if defined(_WIN32) || defined(_WIN64)
#  define FORCE_INLINE __forceinline
#elif defined(linux) || defined(__linux__)
#  define FORCE_INLINE __attribute__((always_inline))
#endif

#define COMMON_TYPE_ALIASING(N, Real_t) \
    using Vec = VecX<N, Real_t>;        \
    using Mat = MatXxX<N, Real_t>;

/****************************************************************************************************/
/* For debug printing */
