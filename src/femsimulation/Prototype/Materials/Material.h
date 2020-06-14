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

#include <SimParams.h>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Vector3.h>

namespace Magnum { namespace Examples {
class Material {
public:
    Material()          = default;
    virtual ~Material() = default;
    virtual Matrix3 computeStressTensor(const Matrix3& F) = 0;
    virtual Float computeEnergy(const Matrix3& /* F */, const Float /* w */) {
        //Fatal() << "Not implemented!"; return 0;
        return 0;
    }

    Float _mu     = 100;
    Float _lambda = 25;
};
} }
