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

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
class CollisionObject {
    COMMON_TYPE_ALIASING(DIM, Real_t)
public:
    CollisionObject()          = default;
    virtual ~CollisionObject() = default;

    virtual void findCollision(const StdVT<Vec>& vertPos, StdVT<std::pair<IntType, Vec>>& collisionInfo) = 0;
    virtual void resolveCollision(StdVT<Vec>& vertPos, StdVT<Vec>& vertVel, Real_t dt) = 0;
    virtual void draw() = 0;
    virtual void updateState(double time) = 0;
};
