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

#include <CollisionObjects/CollisionObjectFactory.h>

#include <SimParams.h>
#include <algorithm>

/****************************************************************************************************/
/* Include all CollisionObject headers */
#include <CollisionObjects/CollisionObject.h>

#include <CollisionObjects/BoxObject.h>

namespace Magnum { namespace Examples {
/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
std::shared_ptr<CollisionObject<DIM, Real_t, IntType>> CollisionObjectFactory<DIM, Real_t, IntType>::create() {
    String CollisionObjectName(g_CollisionObjectType);
    std::transform(CollisionObjectName.begin(), CollisionObjectName.end(), CollisionObjectName.begin(), std::tolower);
    if(CollisionObjectName.find("rectangle") != std::string::npos) {
        return std::dynamic_pointer_cast<CollisionObject<DIM, Real_t, IntType>>(
            std::make_shared<BoxObject<DIM, Real_t, IntType>>());
    }

    //Warning() << "NO CollisionObject generated";
    return nullptr;
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_CollisionObjectFactory(DIM, Real_t, IntType) template class CollisionObjectFactory<DIM, Real_t, IntType>;

INSTANTIATE_CollisionObjectFactory(2, float, u32)
INSTANTIATE_CollisionObjectFactory(3, float, u32)

// INSTANTIATE_CollisionObjectFactory(2, double)
// INSTANTIATE_CollisionObjectFactory(3, double)
} }
