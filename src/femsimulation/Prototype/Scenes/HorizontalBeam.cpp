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

#include <Scenes/HorizontalBeam.h>

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void HorizontalBeam<DIM, Real_t, IntType>::setupScene(StdVT<Vec>&          vertPositions,
                                                      StdVT<IntType>&      fixedVertIndices,
                                                      StdVT<Element<DIM>>& elements) {
    static constexpr Real_t scale = static_cast<Real_t>(0.5);

    if constexpr (DIM == 2) { /* 2D scene */
        static constexpr u32 nRows = 3;
        static constexpr u32 nCols = 10;

        vertPositions.resize((nCols + 1) * (nRows + 1) * DIM);
        u64 idx = 0;
        for(u32 j = 0; j < nCols + 1; ++j) {
            for(u32 i = 0; i < nRows + 1; ++i) {
                vertPositions[idx++] = Vec2r(j,  i) * scale;
            }
        }

        for(u32 j = 0; j < nCols; ++j) {
            for(u32 i = 0; i < nRows; ++i) {
                std::vector<u32> tri0 = { (nRows + 1) * j + i,
                                          (nRows + 1) * j + i + 1,
                                          (nRows + 1) * (j + 1) + i + 1 };
                std::vector<u32> tri1 = { (nRows + 1) * j + i,
                                          (nRows + 1) * (j + 1) + i + 1,
                                          (nRows + 1) * (j + 1) + i };

                elements.push_back(Element<DIM>(tri0, vertPositions));
                elements.push_back(Element<DIM>(tri1, vertPositions));
            }
        }

#if 0
        /* Constrain 1 column of vertices on the left side */
        for(int i = 0; i < nRowsElements + 1; ++i) {
            fixedVertIndices.push_back(static_cast<IntType>(i));
        }
        Debug() << "Initialize scene HorizontalBeam,"
                << nRows - 1 << "layers of element tall, fixed on the left side";
#else
        /* Constrain 2 columns of vertices on the right side */
        const auto last = (nCols + 1) * (nRows + 1) - 1;
        for(u32 i = 0; i < 2 * (nRows + 1); ++i) {
            fixedVertIndices.push_back(static_cast<IntType>(last - i));
        }
        std::sort(fixedVertIndices.begin(), fixedVertIndices.end());

        /* Setup material parameters */
        const Real_t mu = g_bAllowSceneToChangeMaterialStiffnesses ? real(1500) : g_mu;
        this->setMaterialParameters(elements, mu, g_lambda);

        // Debug() << "Initialize scene HorizontalBeam,"
        //         << nRows - 1 << "layers of element tall, fixed on the right side";
#endif
    } /* End 2D scene */
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    else { /* 3D scene */
        static constexpr u32 nCols = 10;
        //        Debug() << "Initialize scene HorizontalBeam, 2 layers of elements tall, fixed on the left side";

        StdVT<Real_t> tmpPos((nCols + 1) * 3 * 9);
        for(u32 i = 0; i < 3; i++) {
            for(u32 j = 0; j < 3; j++) {
                tmpPos[i * 9 + j * 3 + 0] = 0;
                tmpPos[i * 9 + j * 3 + 1] = i * scale;
                tmpPos[i * 9 + j * 3 + 2] = j * scale;
            }
        }

        const u32 floatNumPerLayer = 3 * 9;
        for(u32 l = 1; l < nCols + 1; ++l) {
            for(u32 i = 0; i < 3; i++) {
                for(u32 j = 0; j < 3; j++) {
                    tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 0] = l * scale;
                    tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 1] = i * scale;
                    tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 2] = j * scale;
                }
            }
        }

        const auto numVerts = tmpPos.size() / DIM;
        vertPositions.resize(numVerts);
        for(u64 v = 0; v < numVerts; ++v) {
            vertPositions[v] = Vec(tmpPos[v * 3],
                                   tmpPos[v * 3 + 1],
                                   tmpPos[v * 3 + 2]);
        }

        for(u32 i = 0; i < nCols; i++) {
            for(u32 j = 0; j < 2; j++) {
                for(u32 k = 0; k < 2; k++) {
                    std::vector<u32> tet0, tet1, tet2, tet3, tet4;

                    const u32 indices[8] = {
                        i* 9 + j * 3 + k,                 // 0
                        i* 9 + j * 3 + k + 1,             // 1
                        i* 9 + (j + 1) * 3 + k,           // 2
                        i* 9 + (j + 1) * 3 + k + 1,       // 3
                        (i + 1) * 9 + j * 3 + k,          // 4
                        (i + 1) * 9 + j * 3 + k + 1,      // 5
                        (i + 1) * 9 + (j + 1) * 3 + k,    // 6
                        (i + 1) * 9 + (j + 1) * 3 + k + 1 // 7
                    };

                    tet0.push_back(indices[0]); tet0.push_back(indices[1]); tet0.push_back(indices[3]); tet0.push_back(indices[5]);
                    tet1.push_back(indices[6]); tet1.push_back(indices[7]); tet1.push_back(indices[5]); tet1.push_back(indices[3]);
                    tet2.push_back(indices[0]); tet2.push_back(indices[5]); tet2.push_back(indices[6]); tet2.push_back(indices[3]);
                    tet3.push_back(indices[0]); tet3.push_back(indices[3]); tet3.push_back(indices[6]); tet3.push_back(indices[2]);
                    tet4.push_back(indices[0]); tet4.push_back(indices[4]); tet4.push_back(indices[5]); tet4.push_back(indices[6]);

                    elements.push_back(Element<3>(tet0, vertPositions));
                    elements.push_back(Element<3>(tet1, vertPositions));
                    elements.push_back(Element<3>(tet2, vertPositions));
                    elements.push_back(Element<3>(tet3, vertPositions));
                    elements.push_back(Element<3>(tet4, vertPositions));
                }
            }
        }

        for(u32 i = 0; i < 9; i++) {
            fixedVertIndices.push_back(static_cast<IntType>(i));
        }

        /* Setup material parameters */
        // const Real_t mu = g_bAllowSceneToChangeMaterialStiffnesses ? real(3000) : g_mu; /* for 5 columns */
        const Real_t mu = g_bAllowSceneToChangeMaterialStiffnesses ? real(5000) : g_mu; /* for 10 columns */
        this->setMaterialParameters(elements, mu, g_lambda);
    } /* End 3D scene */
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_HorizontalBeam(DIM, Real_t, IntType) template class HorizontalBeam<DIM, Real_t, IntType>;

INSTANTIATE_HorizontalBeam(2, float, int)
INSTANTIATE_HorizontalBeam(2, float, u32)
INSTANTIATE_HorizontalBeam(3, float, int)
INSTANTIATE_HorizontalBeam(3, float, u32)

// INSTANTIATE_HorizontalBeam(2, double, int)
// INSTANTIATE_HorizontalBeam(2, double, u32)
// INSTANTIATE_HorizontalBeam(3, double, int)
// INSTANTIATE_HorizontalBeam(3, double, u32)
