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

#include <Element.h>
#include <Materials/Material.h>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Vector3.h>

namespace Magnum { namespace Examples {
class Scene {
public:
    Scene()          = default;
    virtual ~Scene() = default;
    virtual void setupScene(std::vector<Vector3>&     vertPositions,
                            std::vector<UnsignedInt>& fixedVertIndices,
                            std::vector<Element>&     elements) = 0;

protected:
    void setMaterialParameters(std::vector<Element>& elements, const Float mu, const Float lambda) {
        for(auto& e: elements) {
            e._material->_mu     = mu;
            e._material->_lambda = lambda;
        }
    }
};
} }
