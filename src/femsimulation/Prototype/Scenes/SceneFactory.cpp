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
#include <string>

/****************************************************************************************************/
/* Include all scene headers */
#include <Scenes/SceneFactory.h>

#include <Scenes/HorizontalBeam.h>

namespace Magnum { namespace Examples {
std::shared_ptr<Scene> SceneFactory::create() {
    std::string sceneName(g_sceneName);
    std::transform(sceneName.begin(), sceneName.end(), sceneName.begin(), std::tolower);
    if(sceneName.find("beam") != std::string::npos) {
        return std::dynamic_pointer_cast<Scene>(
            std::make_shared<HorizontalBeam>());
    }

    return nullptr;
}
} }
