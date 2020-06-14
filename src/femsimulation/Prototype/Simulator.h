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

#include <Attachment.h>
#include <Element.h>

#include <memory>

template<int DIM, class Real_t> class Animation;
template<int DIM, class Real_t, class IntType> class CollisionObject;
/****************************************************************************************************/
template<int DIM>
class Simulator {
    COMMON_TYPE_ALIASING(DIM, real)

public:
    Simulator();

    u64 numVerts() const { return _vertPos.size(); }
    u64 numElements() const { return _elements.size(); }

    bool performGradientDescentOneIteration(StdVT<Vec>& x);
    void evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& _gradient);
    real evaluateEnergy(const StdVT<Vec>& x);
    real lineSearch(const StdVT<Vec>& x, const StdVT<Vec>& gradient_dir, const StdVT<Vec>& descent_dir);

    void step();
    void draw();
    void reset() { _time = 0; _frame = 0; }

    ////////////////////////////////////////////////////////////////////////////////
    StdVT<Element<DIM>>          _elements;
    StdVT<Vec>                   _vertPos;
    StdVT<u32>                   _fixedVerts;
    StdVT<Attachment<DIM, real>> _attachmentConstr;
    StdVT<Vec>                   _gradient;
    StdVT<Vec>                   _externalForces;
    StdVT<Vec>                   _vertPredictedPos;
    StdVT<Vec>                   _oldVertPos;
    StdVT<Vec>                   _vertVel;

    /* Variable for line search */
    inline static constexpr real _ls_alpha = real(1.0);
    inline static constexpr real _ls_beta  = real(0.3);

    inline static constexpr real _mass    = real(1.0);
    inline static constexpr real _massInv = real(1.0);

    /* Frame counter */
    double _time { 0 };
    u64    _frame { 0 };

    std::shared_ptr<Animation<DIM, real>>                   _animation;
    StdVT<std::shared_ptr<CollisionObject<DIM, real, u32>>> _CollisionObjs;
};
