/**
 * Copyright 2020 Nghia Truong <nghiatruong.vn@gmail.com>
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
#ifdef _MSC_VER
#pragma warning (disable: 4127) // condition expression is constant
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': strcpy, strdup, sprintf, vsnprintf, sscanf, fopen
#endif

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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

template<int N, class Type> using VecX   = Eigen::Matrix<Type, N, 1>;
template<int N, class Type> using MatXxX = Eigen::Matrix<Type, N, N>;

/* Fixed size vector/matrix */
using Vec2f = VecX<2, float>;
using Vec3f = VecX<3, float>;
using Vec4f = VecX<4, float>;

using Vec2i = VecX<2, int>;
using Vec3i = VecX<3, int>;
using Vec4i = VecX<4, int>;

using Vec2ui = VecX<2, u32>;
using Vec3ui = VecX<3, u32>;
using Vec4ui = VecX<4, u32>;

using Vec2ul = VecX<2, size_t>;
using Vec3ul = VecX<3, size_t>;
using Vec4ul = VecX<4, size_t>;

using Mat2f = MatXxX<2, float>;
using Mat3f = MatXxX<3, float>;
using Mat4f = MatXxX<4, float>;

using Mat3x4f = Eigen::Matrix<float, 3, 4>;

/* Dynamic size vector/matrix */
using MatX3f = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>; /* must be row major for memcpy */
using VecXf  = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using MatXf  = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

/* Sparse matrix */
using DiagonalMatrixf = Eigen::DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using SparseMatrixf   = Eigen::SparseMatrix<float>;
using Tripletf        = Eigen::Triplet<float, u32>;

/* Access a block of 3 scalars */
#define block3(a) block<3, 1>(3 * (a), 0)

/****************************************************************************************************/
/* For debug printing */
#include <Corrade/Utility/Debug.h>
#include <Magnum/Magnum.h>
using namespace Magnum;

using Debug   = Corrade::Utility::Debug;
using Warning = Corrade::Utility::Warning;
using Error   = Corrade::Utility::Error;
using Fatal   = Corrade::Utility::Fatal;
