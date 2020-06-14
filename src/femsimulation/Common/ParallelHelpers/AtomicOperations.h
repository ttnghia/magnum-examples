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

#include <Setup.h>

#include <atomic>

/****************************************************************************************************/
namespace AtomicOps {
/****************************************************************************************************/
template<class T, class Function>
FORCE_INLINE void atomicOp(T& target, T operand, Function&& f) {
    std::atomic<T>& tgt = *((std::atomic<T>*)&target);

    T cur_val = target;
    T new_val;
    do {
        new_val = f(cur_val, operand);
    } while(!tgt.compare_exchange_weak(cur_val, new_val));
}

/****************************************************************************************************/
template<class T>
FORCE_INLINE void add(T& target, T operand) {
    atomicOp(target, operand, [](T a, T b) { return a + b; });
}

template<i32 N, class T>
FORCE_INLINE void add(VecX<N, T>& target, const VecX<N, T>& operand) {
    static_assert(N == 2 || N == 3);
    auto func = [](T a, T b) { return a + b; };
    if constexpr (N == 2) {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
    } else {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
        atomicOp(target[2], operand[2], func);
    }
}

/****************************************************************************************************/
template<class T>
FORCE_INLINE void subtract(T& target, T operand) {
    atomicOp(target, operand, [](T a, T b) { return a - b; });
}

template<i32 N, class T>
FORCE_INLINE void subtract(VecX<N, T>& target, const VecX<N, T>& operand) {
    static_assert(N == 2 || N == 3);
    auto func = [](T a, T b) { return a - b; };
    if constexpr (N == 2) {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
    } else {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
        atomicOp(target[2], operand[2], func);
    }
}

/****************************************************************************************************/
template<class T>
FORCE_INLINE void multiply(T& target, T operand) {
    atomicOp(target, operand, [](T a, T b) { return a * b; });
}

template<i32 N, class T>
FORCE_INLINE void multiply(VecX<N, T>& target, const VecX<N, T>& operand) {
    static_assert(N == 2 || N == 3);
    auto func = [](T a, T b) { return a * b; };
    if constexpr (N == 2) {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
    } else {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
        atomicOp(target[2], operand[2], func);
    }
}

/****************************************************************************************************/
template<class T>
FORCE_INLINE void divide(T& target, T operand) {
    atomicOp(target, operand, [](T a, T b) { return a / b; });
}

template<i32 N, class T>
FORCE_INLINE void divide(VecX<N, T>& target, const VecX<N, T>& operand) {
    static_assert(N == 2 || N == 3);
    auto func = [](T a, T b) { return a / b; };
    if constexpr (N == 2) {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
    } else {
        atomicOp(target[0], operand[0], func);
        atomicOp(target[1], operand[1], func);
        atomicOp(target[2], operand[2], func);
    }
}

/****************************************************************************************************/
} // end namespace AtomicOps
