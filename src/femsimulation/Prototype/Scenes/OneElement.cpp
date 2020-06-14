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

#include <Scenes/OneElement.h>

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void OneElement<DIM, Real_t, IntType>::setupScene(StdVT<Vec>&          vertPositions,
                                                  StdVT<IntType>&      fixedVertIndices,
                                                  StdVT<Element<DIM>>& elements) {
    Debug() << "Initialize scene OneElement hanging vertically, 1 vertex fixed";
    if constexpr (DIM == 2) {
        vertPositions.resize(0);
        vertPositions.emplace_back(Vec(0, 0));
        vertPositions.emplace_back(Vec(-1, -1));
        vertPositions.emplace_back(Vec(1, -1));

        std::vector<u32> tri0;
        tri0.push_back(0);
        tri0.push_back(1);
        tri0.push_back(2);
        elements.push_back(Element<DIM>(tri0, vertPositions));

        fixedVertIndices.push_back(0);
    } else {
        const Real_t l = static_cast<Real_t>(std::sqrt(3) * 0.5);

        vertPositions.resize(0);
        vertPositions.emplace_back(Vec(0, 0, 0));
        vertPositions.emplace_back(Vec(-l, -1, -0.5));
        vertPositions.emplace_back(Vec(l, -1, -0.5));
        vertPositions.emplace_back(Vec(0, -1, 1));

        std::vector<u32> tri0;
        tri0.push_back(0);
        tri0.push_back(1);
        tri0.push_back(2);
        tri0.push_back(3);
        elements.push_back(Element<DIM>(tri0, vertPositions));

        fixedVertIndices.push_back(0);
    }

    /* Setup material parameters */
    const Real_t mu = g_bAllowSceneToChangeMaterialStiffnesses ? real(300) : g_mu;
    this->setMaterialParameters(elements, mu, g_lambda);
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_OneElement(DIM, Real_t, IntType) template class OneElement<DIM, Real_t, IntType>;

INSTANTIATE_OneElement(2, float, int)
INSTANTIATE_OneElement(2, float, u32)
INSTANTIATE_OneElement(3, float, int)
INSTANTIATE_OneElement(3, float, u32)

// INSTANTIATE_OneElement(2, double, int)
// INSTANTIATE_OneElement(2, double, u32)
// INSTANTIATE_OneElement(3, double, int)
// INSTANTIATE_OneElement(3, double, u32)
