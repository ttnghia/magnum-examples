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

#include <Common/ParallelHelpers/ParallelObjects.h>
#include <Common/ParallelHelpers/Scheduler.h>
#include <Common/Setup.h>

#include <cassert>
#include <vector>

#include <tbb/tbb.h>

/****************************************************************************************************/
namespace ParallelBLAS {
/****************************************************************************************************/
// dot products
//
template<class Real_t>
inline Real_t dotProduct(const StdVT<Real_t>& x, const StdVT<Real_t>& y) {
    REQUIRE(x.size() == y.size());
    ParallelObjects::DotProduct<1, Real_t> pObj(x, y);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    return pObj.getResult();
}

template<int N, class Real_t>
inline Real_t dotProduct(const StdVT<VecX<N, Real_t>>& x, const StdVT<VecX<N, Real_t>>& y) {
    REQUIRE(x.size() == y.size());
    ParallelObjects::DotProduct<N, Real_t> pObj(x, y);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    return pObj.getResult();
}

/****************************************************************************************************/
template<class Real_t>
inline Real_t norm2(const StdVT<Real_t>& x) {
    ParallelObjects::VectorSumSqr<1, Real_t> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    return pObj.getResult();
}

template<int N, class Real_t>
inline Real_t norm2(const StdVT<VecX<N, Real_t>>& x) {
    ParallelObjects::VectorSumSqr<N, Real_t> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    return pObj.getResult();
}

/****************************************************************************************************/
template<class VectorType>
inline StdVT<VectorType> add(const StdVT<VectorType>& x, const StdVT<VectorType>& y) {
    REQUIRE(x.size() == y.size());
    StdVT<VectorType> z(x.size());
    Scheduler::parallel_for(z.size(), [&](size_t i) { z[i] = x[i] + y[i]; });
    return z;
}

template<class VectorType>
inline StdVT<VectorType> minus(const StdVT<VectorType>& x, const StdVT<VectorType>& y) {
    REQUIRE(x.size() == y.size());
    StdVT<VectorType> z(x.size());
    Scheduler::parallel_for(z.size(), [&](size_t i) { z[i] = x[i] - y[i]; });
    return z;
}

/****************************************************************************************************/
// saxpy (y=alpha*x+y)
//
template<class Real_t, class VectorType>
inline void addScaled(Real_t alpha, const StdVT<VectorType>& x, StdVT<VectorType>& y) {
    Scheduler::parallel_for(x.size(), [&, alpha](size_t i) { y[i] += alpha * x[i]; });
}

/****************************************************************************************************/
// y=x+beta*y
//
template<class Real_t, class VectorType>
inline void scaledAdd(Real_t beta, const StdVT<VectorType>& x, StdVT<VectorType>& y) {
    Scheduler::parallel_for(x.size(), [&, beta](size_t i) { y[i] = beta * y[i] + x[i]; });
}

/****************************************************************************************************/
// x *= alpha
//
template<class Real_t, class VectorType>
inline void scale(Real_t alpha, StdVT<VectorType>& x) {
    Scheduler::parallel_for(x.size(), [&, alpha](size_t i) { x[i] *= alpha; });
}

/****************************************************************************************************/
// y = x * alpha
template<class Real_t, class VectorType>
inline StdVT<VectorType> multiply(Real_t alpha, const StdVT<VectorType>& x) {
    StdVT<VectorType> y(x.size());
    Scheduler::parallel_for(x.size(), [&, alpha](size_t i) { y[i] = x[i] * alpha; });
    return y;
}

/****************************************************************************************************/
} // end namespace ParallelBLAS
