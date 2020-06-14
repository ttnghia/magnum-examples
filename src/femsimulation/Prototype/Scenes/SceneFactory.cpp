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

#include <SimParams.h>
#include <algorithm>

/****************************************************************************************************/
/* Include all scene headers */
#include <Scenes/SceneFactory.h>

#include <Scenes/HorizontalBeam.h>
#include <Scenes/OneElement.h>
#include <Scenes/TwoElements.h>

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
std::shared_ptr<Scene<DIM, Real_t, IntType>> SceneFactory<DIM, Real_t, IntType>::create() {
    String sceneName(g_sceneName);
    std::transform(sceneName.begin(), sceneName.end(), sceneName.begin(), std::tolower);
    if(sceneName.find("beam") != std::string::npos) {
        return std::dynamic_pointer_cast<Scene<DIM, Real_t, IntType>>(
            std::make_shared<HorizontalBeam<DIM, Real_t, IntType>>());
    } else if(sceneName.find("one") != std::string::npos) {
        return std::dynamic_pointer_cast<Scene<DIM, Real_t, IntType>>(
            std::make_shared<OneElement<DIM, Real_t, IntType>>());
    } else if(sceneName.find("two") != std::string::npos) {
        return std::dynamic_pointer_cast<Scene<DIM, Real_t, IntType>>(
            std::make_shared<TwoElements<DIM, Real_t, IntType>>());
    }

    Fatal() << "Invalid scene name";
    return nullptr;
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_SceneFactory(DIM, Real_t, IntType) template class SceneFactory<DIM, Real_t, IntType>;

INSTANTIATE_SceneFactory(2, float, int)
INSTANTIATE_SceneFactory(2, float, u32)
INSTANTIATE_SceneFactory(3, float, int)
INSTANTIATE_SceneFactory(3, float, u32)

// INSTANTIATE_SceneFactory(2, double, int)
// INSTANTIATE_SceneFactory(2, double, u32)
// INSTANTIATE_SceneFactory(3, double, int)
// INSTANTIATE_SceneFactory(3, double, u32)
