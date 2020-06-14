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

namespace Magnum { namespace Examples {
void HorizontalBeam::setupScene(std::vector<Vector3>&     vertPositions,
                                std::vector<UnsignedInt>& fixedVertIndices,
                                std::vector<Element>&     elements) {
    static constexpr Float scale = static_cast<Float>(0.5);

    static constexpr UnsignedInt nCols = 10;
    //        Debug() << "Initialize scene HorizontalBeam, 2 layers of elements tall, fixed on the left side";

    std::vector<Float> tmpPos((nCols + 1) * 3 * 9);
    for(UnsignedInt i = 0; i < 3; i++) {
        for(UnsignedInt j = 0; j < 3; j++) {
            tmpPos[i * 9 + j * 3 + 0] = 0;
            tmpPos[i * 9 + j * 3 + 1] = i * scale;
            tmpPos[i * 9 + j * 3 + 2] = j * scale;
        }
    }

    const UnsignedInt floatNumPerLayer = 3 * 9;
    for(UnsignedInt l = 1; l < nCols + 1; ++l) {
        for(UnsignedInt i = 0; i < 3; i++) {
            for(UnsignedInt j = 0; j < 3; j++) {
                tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 0] = l * scale;
                tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 1] = i * scale;
                tmpPos[floatNumPerLayer * l + i * 9 + j * 3 + 2] = j * scale;
            }
        }
    }

    const auto numVerts = tmpPos.size() / 3;
    vertPositions.resize(numVerts);
    for(std::size_t v = 0; v < numVerts; ++v) {
        vertPositions[v] = Vector3(tmpPos[v * 3],
                                   tmpPos[v * 3 + 1],
                                   tmpPos[v * 3 + 2]);
    }

    for(UnsignedInt i = 0; i < nCols; i++) {
        for(UnsignedInt j = 0; j < 2; j++) {
            for(UnsignedInt k = 0; k < 2; k++) {
                std::vector<UnsignedInt> tet0, tet1, tet2, tet3, tet4;

                const UnsignedInt indices[8] = {
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

                elements.push_back(Element(tet0, vertPositions));
                elements.push_back(Element(tet1, vertPositions));
                elements.push_back(Element(tet2, vertPositions));
                elements.push_back(Element(tet3, vertPositions));
                elements.push_back(Element(tet4, vertPositions));
            }
        }
    }

    for(UnsignedInt i = 0; i < 9; i++) {
        fixedVertIndices.push_back(static_cast<UnsignedInt>(i));
    }

    /* Setup material parameters */
    // const Float mu = g_bAllowSceneToChangeMaterialStiffnesses ? Float(3000) : g_mu; /* for 5 columns */
    const Float mu = g_bAllowSceneToChangeMaterialStiffnesses ? Float(5000) : g_mu; /* for 10 columns */
    this->setMaterialParameters(elements, mu, g_lambda);
}
} }
