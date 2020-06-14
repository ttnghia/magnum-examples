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

#include <Animations/AnimationFactory.h>

#include <SimParams.h>
#include <algorithm>

/****************************************************************************************************/
/* Include all Animation headers */
#include <Animations/Animation.h>

#include <Animations/DownUp.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
std::shared_ptr<Animation<DIM, Real_t>> AnimationFactory<DIM, Real_t>::create() {
    String animationName(g_AnimationType);
    std::transform(animationName.begin(), animationName.end(), animationName.begin(), std::tolower);
    if(animationName.find("down-up") != std::string::npos) {
        return std::dynamic_pointer_cast<Animation<DIM, Real_t>>(
            std::make_shared<DownUp<DIM, Real_t>>());
    }

    Fatal() << "Invalid animation type";
    return nullptr;
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_AnimationFactory(DIM, Real_t) template class AnimationFactory<DIM, Real_t>;

INSTANTIATE_AnimationFactory(2, float)
INSTANTIATE_AnimationFactory(3, float)

// INSTANTIATE_AnimationFactory(2, double)
// INSTANTIATE_AnimationFactory(3, double)
