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
#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Vector3.h>

#include <vector>
namespace Magnum { namespace Examples {
/****************************************************************************************************/

class Attachment {
public:
    Attachment(const UnsignedInt idx, const std::vector<Vector3>& x) : _fixed_pos(x[idx]), _idx(idx) {}

    void  evaluateGradient(const std::vector<Vector3>& x, std::vector<Vector3>& gradient);
    Float evaluateEnergy(const std::vector<Vector3>& x);

    UnsignedInt getVertexIdx() const { return _idx; }
    Vector3 getFixedPosition() const { return _fixed_pos; }
    void setFixedPosition(const Vector3& pos) { _fixed_pos = pos; }

private:
    Vector3 _fixed_pos; /* can be changed for animation */

    const UnsignedInt             _idx;
    inline static constexpr Float _stiffness = 1e6f;
};
} }