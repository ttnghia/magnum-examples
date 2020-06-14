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

/****************************************************************************************************/
template<int DIM, class Real_t>
class Attachment {
    COMMON_TYPE_ALIASING(DIM, Real_t)

public:
    Attachment(const u32 idx, const StdVT<Vec>& x) : _fixed_pos(x[idx]), _idx(idx) {}

    void   evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& gradient);
    Real_t evaluateEnergy(const StdVT<Vec>& x);

    u32 getVertexIdx() const { return _idx; }
    Vec getFixedPosition() const { return _fixed_pos; }
    void setFixedPosition(const Vec& pos) { _fixed_pos = pos; }

private:
    Vec _fixed_pos; /* can be changed for animation */

    const u32 _idx;
    inline static constexpr Real_t _stiffness = 1e6f;
};
