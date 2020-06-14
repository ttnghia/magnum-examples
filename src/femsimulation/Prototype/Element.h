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

template<int DIM, class Real_t> class Material;
/****************************************************************************************************/
template<int DIM>
class Element {
    COMMON_TYPE_ALIASING(DIM, real)

public:
    Element(const StdVT<u32> idx, const StdVT<Vec>& x);

    void evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& gradient);
    real evaluateEnergy(const StdVT<Vec>& x);

    Mat  getMatrixDs(const StdVT<Vec>& x);
    Mat  getDeformationGradient(const StdVT<Vec>& x);
    real getTorqueMagnitude(const Mat& H) const;
    void updateElementWeight();

    void draw(StdVT<Vec>& p);

    StdVT<u32> _idx;

    Mat  _Dm;
    Mat  _Dm_inv;
    Mat  _Dm_invT;
    real _w;

    std::shared_ptr<Material<DIM, real>> _material;
};
