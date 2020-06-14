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

#include <memory>
#include <vector>
namespace Magnum { namespace Examples {
class Material;
/****************************************************************************************************/
class Element {
public:
    Element(const std::vector<UnsignedInt> idx, const std::vector<Vector3>& x);

    void  evaluateGradient(const std::vector<Vector3>& x, std::vector<Vector3>& gradient);
    Float evaluateEnergy(const std::vector<Vector3>& x);

    Matrix3 getMatrixDs(const std::vector<Vector3>& x);
    Matrix3 getDeformationGradient(const std::vector<Vector3>& x);
    void    updateElementWeight();

    void draw(std::vector<Vector3>& p);

    std::vector<UnsignedInt> _idx;

    Matrix3 _Dm;
    Matrix3 _Dm_inv;
    Matrix3 _Dm_invT;
    Float   _w;

    std::shared_ptr<Material> _material;
};
} }
