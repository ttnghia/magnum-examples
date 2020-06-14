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

#include <cassert>
#include <vector>
#include <cmath>

#include <tbb/tbb.h>
#include <Common/ParallelHelpers/ParallelObjects.h>

/****************************************************************************************************/
namespace ParallelSTL {
/****************************************************************************************************/
// min/max element
template<class T>
inline T min(const StdVT<T>& x) {
    ParallelObjects::MinElement<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<int N, class T>
inline T min(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::MinElement<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

/****************************************************************************************************/
template<class T>
inline T max(const StdVT<T>& x) {
    ParallelObjects::MaxElement<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<int N, class T>
inline T max(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::MaxElement<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

/****************************************************************************************************/
template<class T>
inline T maxAbs(const StdVT<T>& x) {
    ParallelObjects::MaxAbs<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<int N, class T>
inline T maxAbs(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::MaxAbs<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

/****************************************************************************************************/
template<int N, class T>
inline T maxNorm2(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::MaxNorm2<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

/****************************************************************************************************/
template<class T>
inline void min_max(const StdVT<T>& x, T& minElement, T& maxElement) {
    ParallelObjects::MinMaxElements<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    minElement = pObj.getMin();
    maxElement = pObj.getMax();
}

template<int N, class T>
inline void min_max(const StdVT<VecX<N, T>>& x, VecX<N, T>& minElement, VecX<N, T>& maxElement) {
    ParallelObjects::MinMaxElements<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    minElement = pObj.getMin();
    maxElement = pObj.getMax();
}

template<int N, class T>
inline void min_max(const StdVT<MatXxX<N, T>>& x, T& minElement, T& maxElement) {
    ParallelObjects::MinMaxElements<0, T> pObj(reinterpret_cast<const T*>(x.data()));
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size() * static_cast<size_t>(N * N)), pObj);

    minElement = pObj.getMin();
    maxElement = pObj.getMax();
}

template<int N, class T>
inline void min_max_norm2(const StdVT<VecX<N, T>>& x, T& minVal, T& maxVal) {
    ParallelObjects::MinMaxNorm2<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);

    minVal = pObj.getMin();
    maxVal = pObj.getMax();
}

/****************************************************************************************************/
template<class T>
inline T sum(const StdVT<T>& x) {
    ParallelObjects::VectorSum<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<int N, class T>
inline VecX<N, T> sum(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::VectorSum<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<class T>
inline T sum_sqr(const StdVT<T>& x) {
    ParallelObjects::VectorSumSqr<1, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<int N, class T>
inline T sum_sqr(const StdVT<VecX<N, T>>& x) {
    ParallelObjects::VectorSumSqr<N, T> pObj(x);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, x.size()), pObj);
    return pObj.getResult();
}

template<class T>
inline T average(const StdVT<T>& x) {
    return ParallelSTL::sum<T>(x) / static_cast<T>(x.size());
}

/****************************************************************************************************/
// sorting
template<class T>
inline void sort(StdVT<T>& v) {
    tbb::parallel_sort(v);
}

template<class T>
inline void sort_dsd(StdVT<T>& v) {
    tbb::parallel_sort(std::begin(v), std::end(v), std::greater<T> ());
}

/****************************************************************************************************/
} // end namespace ParallelSTL
