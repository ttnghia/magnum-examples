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

#include <Scenes/TwoElements.h>

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void TwoElements<DIM, Real_t, IntType>::setupScene(StdVT<Vec>&          vertPositions,
                                                   StdVT<IntType>&      fixedVertIndices,
                                                   StdVT<Element<DIM>>& elements) {
    if constexpr (DIM == 2) {
#  if 1
        Debug() << "Initialize scene TwoElements hanging vertically, 1 vertex fixed";
        vertPositions.resize(0);
        vertPositions.emplace_back(Vec(0, 0));
        vertPositions.emplace_back(Vec(-1, -1));
        vertPositions.emplace_back(Vec(1, -1));
        vertPositions.emplace_back(Vec(0, -2));

        std::vector<u32> tri0, tri1;
        tri0.push_back(0);
        tri0.push_back(1);
        tri0.push_back(2);
        tri1.push_back(1);
        tri1.push_back(2);
        tri1.push_back(3);
        elements.push_back(Element<DIM>(tri0, vertPositions));
        elements.push_back(Element<DIM>(tri1, vertPositions));

        fixedVertIndices.push_back(0);

#  else
        Debug() << "Initialize scene TwoElements hanging horizontally, 2 vertices fixed";

        vertPositions.resize(0);
        vertPositions.emplace_back(Vec(0, 0));
        vertPositions.emplace_back(Vec(0, 1));
        vertPositions.emplace_back(Vec(1, 1));
        vertPositions.emplace_back(Vec(1, 0));

        StdVT<u32> tri0, tri1;
        tri0.push_back(0);
        tri0.push_back(1);
        tri0.push_back(2);
        tri1.push_back(0);
        tri1.push_back(2);
        tri1.push_back(3);
        elements.push_back(Element<DIM>(tri0, vertPositions));
        elements.push_back(Element<DIM>(tri1, vertPositions));

        fixedVertIndices.push_back(2);
        fixedVertIndices.push_back(3);

#  endif
    } else {
        Debug() << "Initialize scene TwoElements hanging vertically, 1 vertex fixed";
        vertPositions.resize(0);
        vertPositions.emplace_back(Vec(0, 0, 0));
        vertPositions.emplace_back(Vec(-1, -1, 0));
        vertPositions.emplace_back(Vec(0, -1, 1));
        vertPositions.emplace_back(Vec(1, -1, 0));
        vertPositions.emplace_back(Vec(0, -1, -1));

        std::vector<u32> tri0, tri1;
        tri0.push_back(0);
        tri0.push_back(1);
        tri0.push_back(2);
        tri0.push_back(3);
        tri1.push_back(0);
        tri1.push_back(1);
        tri1.push_back(3);
        tri1.push_back(4);
        elements.push_back(Element<DIM>(tri0, vertPositions));
        elements.push_back(Element<DIM>(tri1, vertPositions));

        fixedVertIndices.push_back(0);
    }

    /* Setup material parameters */
    const Real_t mu = g_bAllowSceneToChangeMaterialStiffnesses ? real(300) : g_mu;
    this->setMaterialParameters(elements, mu, g_lambda);
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_TwoElements(DIM, Real_t, IntType) template class TwoElements<DIM, Real_t, IntType>;

INSTANTIATE_TwoElements(2, float, int)
INSTANTIATE_TwoElements(2, float, u32)
INSTANTIATE_TwoElements(3, float, int)
INSTANTIATE_TwoElements(3, float, u32)

// INSTANTIATE_TwoElements(2, double, int)
// INSTANTIATE_TwoElements(2, double, u32)
// INSTANTIATE_TwoElements(3, double, int)
// INSTANTIATE_TwoElements(3, double, u32)
