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

#include <Common/Setup.h>

#include <utility>
#include <algorithm>
#include <functional>

#define __NT_NO_PARALLEL

/****************************************************************************************************/
namespace Scheduler {
/****************************************************************************************************/
template<class IndexType, class Function>
FORCE_INLINE void parallel_for(IndexType beginIdx, IndexType endIdx, Function&& function) {
#if defined(__NT_NO_PARALLEL) || defined(__NT_DISABLE_PARALLEL)
    for(IndexType i = beginIdx; i < endIdx; ++i) {
        function(i);
    }
#else
    tbb::parallel_for(tbb::blocked_range<IndexType>(beginIdx, endIdx),
                      [&](const tbb::blocked_range<IndexType>& r) {
                          for(IndexType i = r.begin(), iEnd = r.end(); i < iEnd; ++i) {
                              function(i);
                          }
                      });
#endif
}

template<class IndexType, class Function>
FORCE_INLINE void parallel_for(IndexType endIdx, Function&& function) {
    Scheduler::parallel_for(IndexType(0), endIdx, std::forward<Function>(function));
}

/****************************************************************************************************/
// parallel for 2D
template<class IndexType, class Function>
FORCE_INLINE void parallel_for(IndexType beginIdxX, IndexType endIdxX,
                               IndexType beginIdxY, IndexType endIdxY,
                               Function&& function) {
    Scheduler::parallel_for(beginIdxY, endIdxY,
                            [&](IndexType j) {
                                for(IndexType i = beginIdxX; i < endIdxX; ++i) {
                                    function(i, j);
                                }
                            });
}

template<class IndexType, class Function>
FORCE_INLINE void parallel_for(const Vec2<IndexType>& endIdx, Function&& function) {
    Scheduler::parallel_for(IndexType(0), endIdx[0], IndexType(0), endIdx[1],
                            std::forward<Function>(function));
}

/****************************************************************************************************/
// parallel for 3D
template<class IndexType, class Function>
FORCE_INLINE void parallel_for(IndexType beginIdxX, IndexType endIdxX,
                               IndexType beginIdxY, IndexType endIdxY,
                               IndexType beginIdxZ, IndexType endIdxZ,
                               Function&& function) {
    Scheduler::parallel_for(beginIdxZ, endIdxZ,
                            [&](IndexType k) {
                                for(IndexType j = beginIdxY; j < endIdxY; ++j) {
                                    for(IndexType i = beginIdxX; i < endIdxX; ++i) {
                                        function(i, j, k);
                                    }
                                }
                            });
}

template<class IndexType, class Function>
FORCE_INLINE void parallel_for(const Vec3<IndexType>& endIdx, Function&& function) {
    Scheduler::parallel_for(IndexType(0), endIdx[0], IndexType(0), endIdx[1], IndexType(0), endIdx[2],
                            std::forward<Function>(function));
}

/****************************************************************************************************/
} // namespace Scheduler
