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

#include <atomic>
#include <limits>
#include <tbb/tbb.h>

#include <Common/Setup.h>

#undef min
#undef max

/****************************************************************************************************/
namespace ParallelObjects {
/****************************************************************************************************/
class SpinLock {
public:
    SpinLock() = default;
    SpinLock(const SpinLock&) {}
    SpinLock& operator=(const SpinLock&) { return *this; }
    void lock() { while(m_Lock.test_and_set(std::memory_order_acquire)) {} }
    void unlock() { m_Lock.clear(std::memory_order_release); }
private:
    std::atomic_flag m_Lock = ATOMIC_FLAG_INIT;
};

/****************************************************************************************************/
template<int N, class Real_t>
class DotProduct {
public:
    DotProduct(const StdVT<VecX<N, Real_t>>& vec1, const StdVT<VecX<N, Real_t>>& vec2) : m_Vec1(vec1), m_Vec2(vec2) {}
    DotProduct(DotProduct<N, Real_t>& pObj, tbb::split) : m_Vec1(pObj.m_Vec1), m_Vec2(pObj.m_Vec2) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vec1[i].dot(m_Vec2[i]);
        }
    }

    void     join(DotProduct<N, Real_t>& pObj) { m_Result += pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t m_Result = 0;
    const StdVT<VecX<N, Real_t>>& m_Vec1;
    const StdVT<VecX<N, Real_t>>& m_Vec2;
};

////////////////////////////////////////////////////////////////////////////////
template<class Real_t>
class DotProduct<1, Real_t> {
public:
    DotProduct(const StdVT<Real_t>& vec1, const StdVT<Real_t>& vec2) : m_Vec1(vec1), m_Vec2(vec2) {}
    DotProduct(DotProduct<1, Real_t>& pObj, tbb::split) : m_Vec1(pObj.m_Vec1), m_Vec2(pObj.m_Vec2) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vec1[i] * m_Vec2[i];
        }
    }

    void     join(DotProduct<1, Real_t>& pObj) { m_Result += pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t               m_Result = 0;
    const StdVT<Real_t>& m_Vec1;
    const StdVT<Real_t>& m_Vec2;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MinElement {
public:
    MinElement(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {}
    MinElement(MinElement<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            const auto val = m_Vector[i];
            for(int j = 0; j < N; ++j) {
                Real_t tmp = val[j];
                m_Result = m_Result < tmp ? m_Result : tmp;
            }
        }
    }

    void     join(MinElement<N, Real_t>& pObj) { m_Result = m_Result < pObj.m_Result ? m_Result : pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t m_Result = std::numeric_limits<Real_t>::max();
    const StdVT<VecX<N, Real_t>>& m_Vector;
};

////////////////////////////////////////////////////////////////////////////////
template<class Real_t>
class MinElement<1, Real_t> {
public:
    MinElement(const StdVT<Real_t>& vec) : m_Vector(vec) {}
    MinElement(MinElement<1, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result = m_Result < m_Vector[i] ? m_Result : m_Vector[i];
        }
    }

    void     join(MinElement<1, Real_t>& pObj) { m_Result = m_Result < pObj.m_Result ? m_Result : pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t               m_Result = std::numeric_limits<Real_t>::max();
    const StdVT<Real_t>& m_Vector;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MaxElement {
public:
    MaxElement(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {}
    MaxElement(MaxElement<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            const auto val = m_Vector[i];
            for(int j = 0; j < N; ++j) {
                Real_t tmp = val[j];
                m_Result = m_Result > tmp ? m_Result : tmp;
            }
        }
    }

    void     join(MaxElement<N, Real_t>& pObj) { m_Result = m_Result > pObj.m_Result ? m_Result : pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t m_Result = -std::numeric_limits<Real_t>::max();
    const StdVT<VecX<N, Real_t>>& m_Vector;
};

////////////////////////////////////////////////////////////////////////////////
template<class Real_t>
class MaxElement<1, Real_t> {
public:
    MaxElement(const StdVT<Real_t>& vec) : m_Vector(vec) {}
    MaxElement(MaxElement<1, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result = m_Result > m_Vector[i] ? m_Result : m_Vector[i];
        }
    }

    void     join(MaxElement<1, Real_t>& pObj) { m_Result = m_Result > pObj.m_Result ? m_Result : pObj.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t               m_Result = -std::numeric_limits<Real_t>::max();
    const StdVT<Real_t>& m_Vector;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MaxAbs {
public:
    MaxAbs(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {}
    MaxAbs(MaxAbs<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            const auto val = m_Vector[i];
            for(int j = 0; j < N; ++j) {
                Real_t tmp = std::abs(val[j]);
                m_Result = m_Result > tmp ? m_Result : tmp;
            }
        }
    }

    void     join(MaxAbs<N, Real_t>& vma) { m_Result = m_Result > vma.m_Result ? m_Result : vma.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t m_Result = 0;
    const StdVT<VecX<N, Real_t>>& m_Vector;
};

////////////////////////////////////////////////////////////////////////////////
template<class Real_t>
class MaxAbs<1, Real_t> {
public:
    MaxAbs(const StdVT<Real_t>& vec) : m_Vector(vec) {}
    MaxAbs(MaxAbs<1, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            Real_t tmp = fabs(m_Vector[i]);
            m_Result = m_Result > tmp ? m_Result : tmp;
        }
    }

    void     join(MaxAbs<1, Real_t>& vma) { m_Result = m_Result > vma.m_Result ? m_Result : vma.m_Result; }
    Real_t getResult() const noexcept { return m_Result; }

private:
    Real_t               m_Result = 0;
    const StdVT<Real_t>& m_Vector;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MaxNorm2 {
public:
    MaxNorm2(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {}
    MaxNorm2(MaxNorm2<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            Real_t mag2 = m_Vector[i].squaredNorm();
            m_Result = m_Result > mag2 ? m_Result : mag2;
        }
    }

    void     join(MaxNorm2<N, Real_t>& pObj) { m_Result = m_Result > pObj.m_Result ? m_Result : pObj.m_Result; }
    Real_t getResult() const noexcept { return sqrt(m_Result); }

private:
    Real_t m_Result = 0;
    const StdVT<VecX<N, Real_t>>& m_Vector;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MinMaxElements {
public:
    MinMaxElements(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {
        if(vec.size() > 0) { m_ResultMax = vec[0]; }
    }

    MinMaxElements(MinMaxElements<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            const auto& vec = m_Vector[i];
            for(int j = 0; j < N; ++j) {
                m_ResultMin[j] = (m_ResultMin[j] < vec[j]) ? m_ResultMin[j] : vec[j];
                m_ResultMax[j] = (m_ResultMax[j] > vec[j]) ? m_ResultMax[j] : vec[j];
            }
        }
    }

    void join(MinMaxElements<N, Real_t>& pObj) {
        for(int j = 0; j < N; ++j) {
            m_ResultMin[j] = (m_ResultMin[j] < pObj.m_ResultMin[j]) ? m_ResultMin[j] : pObj.m_ResultMin[j];
            m_ResultMax[j] = (m_ResultMax[j] > pObj.m_ResultMax[j]) ? m_ResultMax[j] : pObj.m_ResultMax[j];
        }
    }

    const VecX<N, Real_t>& getMin() const noexcept { return m_ResultMin; }
    const VecX<N, Real_t>& getMax() const noexcept { return m_ResultMax; }

private:
    VecX<N, Real_t>               m_ResultMin = VecX<N, Real_t>(std::numeric_limits<Real_t>::max());
    VecX<N, Real_t>               m_ResultMax = VecX<N, Real_t>(0);
    const StdVT<VecX<N, Real_t>>& m_Vector;
};

////////////////////////////////////////////////////////////////////////////////
// N = 0 : plain C array
template<class Real_t>
class MinMaxElements<0, Real_t> {
public:
    MinMaxElements(const Real_t* vec) : m_Vector(vec) {
        m_ResultMax = vec[0];
    }

    MinMaxElements(MinMaxElements<0, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_ResultMin = m_ResultMin < m_Vector[i] ? m_ResultMin : m_Vector[i];
            m_ResultMax = m_ResultMax > m_Vector[i] ? m_ResultMax : m_Vector[i];
        }
    }

    void join(MinMaxElements<0, Real_t>& pObj) {
        m_ResultMin = m_ResultMin < pObj.m_ResultMin ? m_ResultMin : pObj.m_ResultMin;
        m_ResultMax = m_ResultMax > pObj.m_ResultMax ? m_ResultMax : pObj.m_ResultMax;
    }

    Real_t getMin() const noexcept { return m_ResultMin; }
    Real_t getMax() const noexcept { return m_ResultMax; }

private:
    Real_t m_ResultMin = std::numeric_limits<Real_t>::max();
    Real_t m_ResultMax = 0;

    const Real_t* m_Vector;
};

////////////////////////////////////////////////////////////////////////////////
template<class Real_t>
class MinMaxElements<1, Real_t> {
public:
    MinMaxElements(const StdVT<Real_t>& vec) : m_Vector(vec) {
        if(vec.size() > 0) { m_ResultMax = vec[0]; }
    }

    MinMaxElements(MinMaxElements<1, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_ResultMin = m_ResultMin < m_Vector[i] ? m_ResultMin : m_Vector[i];
            m_ResultMax = m_ResultMax > m_Vector[i] ? m_ResultMax : m_Vector[i];
        }
    }

    void join(MinMaxElements<1, Real_t>& pObj) {
        m_ResultMin = m_ResultMin < pObj.m_ResultMin ? m_ResultMin : pObj.m_ResultMin;
        m_ResultMax = m_ResultMax > pObj.m_ResultMax ? m_ResultMax : pObj.m_ResultMax;
    }

    Real_t getMin() const noexcept { return m_ResultMin; }
    Real_t getMax() const noexcept { return m_ResultMax; }

private:
    Real_t m_ResultMin = std::numeric_limits<Real_t>::max();
    Real_t m_ResultMax = 0;

    const StdVT<Real_t>& m_Vector;
};

/****************************************************************************************************/
template<int N, class Real_t>
class MinMaxNorm2 {
public:
    MinMaxNorm2(const StdVT<VecX<N, Real_t>>& vec) : m_Vector(vec) {}
    MinMaxNorm2(MinMaxNorm2<N, Real_t>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            Real_t mag2 = m_Vector[i].squaredNorm();
            m_ResultMin = m_ResultMin < mag2 ? m_ResultMin : mag2;
            m_ResultMax = m_ResultMax > mag2 ? m_ResultMax : mag2;
        }
    }

    void join(MinMaxNorm2<N, Real_t>& pObj) {
        m_ResultMin = m_ResultMin < pObj.m_ResultMin ? m_ResultMin : pObj.m_ResultMin;
        m_ResultMax = m_ResultMax > pObj.m_ResultMax ? m_ResultMax : pObj.m_ResultMax;
    }

    Real_t getMin() const noexcept { return sqrt(m_ResultMin); }
    Real_t getMax() const noexcept { return sqrt(m_ResultMax); }

private:
    Real_t m_ResultMin = std::numeric_limits<Real_t>::max();
    Real_t m_ResultMax = 0;

    const StdVT<VecX<N, Real_t>>& m_Vector;
};

/****************************************************************************************************/
template<int N, class T>
class VectorSum {
public:
    VectorSum(const StdVT<VecX<N, T>>& vec) : m_Vector(vec) {}
    VectorSum(VectorSum<N, T>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vector[i];
        }
    }

    void              join(VectorSum<N, T>& pObj) { m_Result += pObj.m_Result; }
    const VecX<N, T>& getResult() const noexcept { return m_Result; }
private:
    VecX<N, T>               m_Result = VecX<N, T>(0);
    const StdVT<VecX<N, T>>& m_Vector;
};

template<class T>
class VectorSum<1, T> {
public:
    VectorSum(const StdVT<T>& vec) : m_Vector(vec) {}
    VectorSum(VectorSum<1, T>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vector[i];
        }
    }

    void join(VectorSum<1, T>& pObj) { m_Result += pObj.m_Result; }
    T    getResult() const noexcept { return m_Result; }

private:
    T               m_Result = T(0);
    const StdVT<T>& m_Vector;
};

/****************************************************************************************************/
template<int N, class T>
class VectorSumSqr {
public:
    VectorSumSqr(const StdVT<VecX<N, T>>& vec) : m_Vector(vec) {}
    VectorSumSqr(VectorSumSqr<N, T>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vector[i].squaredNorm();
        }
    }

    void join(VectorSumSqr<N, T>& pObj) { m_Result += pObj.m_Result; }
    T    getResult() const noexcept { return m_Result; }
private:
    T m_Result = T(0);
    const StdVT<VecX<N, T>>& m_Vector;
};

template<class T>
class VectorSumSqr<1, T> {
public:
    VectorSumSqr(const StdVT<T>& vec) : m_Vector(vec) {}
    VectorSumSqr(VectorSumSqr<1, T>& pObj, tbb::split) : m_Vector(pObj.m_Vector) {}

    void operator()(const tbb::blocked_range<size_t>& r) {
        for(size_t i = r.begin(); i != r.end(); ++i) {
            m_Result += m_Vector[i] * m_Vector[i];
        }
    }

    void join(VectorSumSqr<1, T>& pObj) { m_Result += pObj.m_Result; }
    T    getResult() const noexcept { return m_Result; }

private:
    T               m_Result = T(0);
    const StdVT<T>& m_Vector;
};

/****************************************************************************************************/
} // end namespace ParallelObjects
