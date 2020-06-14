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

#include <CollisionObjects/CollisionObject.h>
namespace Magnum { namespace Examples {
/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
class BoxObject : public CollisionObject<DIM, Real_t, IntType> {
    COMMON_TYPE_ALIASING(DIM, Real_t)
public:
    BoxObject();
    virtual void findCollision(const StdVT<Vec>& vertPos, StdVT<std::pair<IntType, Vec>>& collisionInfo) override;
    virtual void resolveCollision(StdVT<Vec>& vertPos, StdVT<Vec>& vertVel, Real_t dt) override;
    virtual void draw() override;
    virtual void updateState(double time) override;

private:
    Vec        _center = Vec::Zero();
    Vec        _halfLengths;
    StdVT<Vec> _corners;

    bool _bEnable { true };
};
} }
