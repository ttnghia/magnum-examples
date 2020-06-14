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

#include <CollisionObjects/BoxObject.h>

#include <Magnum/Magnum.h>
#include <Magnum/GL/GL.h>
#include <Magnum/GL/OpenGL.h>
namespace Magnum { namespace Examples {
/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
BoxObject<DIM, Real_t, IntType>::BoxObject() {
    if constexpr (DIM == 2) {
        _center      = Vec(0.25, -0.25);
        _halfLengths = Vec(0.5, 0.25);

        const Vec offset[4] = {
            Vec(-1, -1),
            Vec( 1, -1),
            Vec( 1,  1),
            Vec(-1,  1)
        };
        for(u64 i = 0; i < 4; ++i) {
            _corners.emplace_back(Vec(offset[i][0] * _halfLengths[0],
                                      offset[i][1] * _halfLengths[1]) + _center);
        }
    } else {
        //        _center      = Vec(2, -0.25, 0.5); /* for 5 columns element */
        _center      = Vec(4, -0.25, 0.5);  /* for 10 columns element */
        _halfLengths = Vec(0.25, 0.25, 1);

        const Vec offset[8] = {
            Vec(-1, -1, -1),
            Vec( 1, -1, -1),
            Vec( 1, -1,  1),
            Vec(-1, -1,  1),

            Vec(-1,  1, -1),
            Vec( 1,  1, -1),
            Vec( 1,  1,  1),
            Vec(-1,  1,  1),
        };
        for(u64 i = 0; i < 8; ++i) {
            _corners.emplace_back(Vec(offset[i][0] * _halfLengths[0],
                                      offset[i][1] * _halfLengths[1],
                                      offset[i][2] * _halfLengths[2]) + _center);
        }
    }
}

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void BoxObject<DIM, Real_t, IntType>::findCollision(const StdVT<Vec>& vertPos,
                                                    StdVT<std::pair<IntType, Vec>>& collisionInfo) {
    if(!_bEnable) {
        return;
    }

    for(u32 v = 0; v < vertPos.size(); ++v) {
        const auto& ppos       = vertPos[v];
        bool        bColliding = true;
        for(int i = 0; i < DIM; ++i) {
            if(std::abs(ppos[i] - _center[i]) > _halfLengths[i]) {
                bColliding = false;
                break;
            }
        }

        if(!bColliding) {
            continue;
        }

        Vec normal = Vec::Zero();
        normal[1] = 1;
        collisionInfo.emplace_back(std::make_pair(v, normal));
    }
}

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void BoxObject<DIM, Real_t, IntType>::resolveCollision(StdVT<Vec>& vertPos, StdVT<Vec>& vertVel, Real_t dt) {
    if(!_bEnable) {
        return;
    }

    for(u32 v = 0; v < vertPos.size(); ++v) {
        auto& ppos       = vertPos[v];
        bool  bColliding = true;
        for(int i = 0; i < DIM; ++i) {
            if(std::abs(ppos[i] - _center[i]) > _halfLengths[i]) {
                bColliding = false;
                break;
            }
        }

        if(!bColliding) {
            continue;
        }

        static constexpr Real_t damping = Real_t(0.01);
        vertVel[v][1] += (_center[1] + _halfLengths[1] - ppos[1]) / dt * damping;
        ppos[1]        = _center[1] + _halfLengths[1];
    }
}

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void BoxObject<DIM, Real_t, IntType>::updateState(double time) {
    if(g_bRemoveContact && time >= g_RemoveContactTime) {
        _bEnable = false;
    }
}

/****************************************************************************************************/
template<int DIM, class Real_t, class IntType>
void BoxObject<DIM, Real_t, IntType>::draw() {
    if(!_bEnable) {
        return;
    }
#if 0
    glColor3f(1, 0, 1);
    glBegin(GL_LINES);
    if constexpr (DIM == 2) {
        if constexpr (sizeof(real) == sizeof(float)) {
            glVertex2fv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex2fv(reinterpret_cast<Real_t*>(&_corners[1]));
            glVertex2fv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex2fv(reinterpret_cast<Real_t*>(&_corners[2]));
            glVertex2fv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex2fv(reinterpret_cast<Real_t*>(&_corners[3]));
            glVertex2fv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex2fv(reinterpret_cast<Real_t*>(&_corners[0]));
        } else {
            glVertex2dv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex2dv(reinterpret_cast<Real_t*>(&_corners[1]));
            glVertex2dv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex2dv(reinterpret_cast<Real_t*>(&_corners[2]));
            glVertex2dv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex2dv(reinterpret_cast<Real_t*>(&_corners[3]));
            glVertex2dv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex2dv(reinterpret_cast<Real_t*>(&_corners[0]));
        }
    } else {
        if constexpr (sizeof(real) == sizeof(float)) {
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[1]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[2]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[3]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[0]));

            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[4])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[5]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[5])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[6]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[6])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[7]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[7])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[4]));

            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[4]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[5]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[6]));
            glVertex3fv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex3fv(reinterpret_cast<Real_t*>(&_corners[7]));
        } else {
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[1]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[2]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[3]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[0]));

            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[4])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[5]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[5])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[6]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[6])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[7]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[7])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[4]));

            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[0])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[4]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[1])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[5]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[2])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[6]));
            glVertex3dv(reinterpret_cast<Real_t*>(&_corners[3])); glVertex3dv(reinterpret_cast<Real_t*>(&_corners[7]));
        }
    }
    glEnd();
#endif
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_BoxObject(DIM, Real_t, IntType) template class BoxObject<DIM, Real_t, IntType>;

INSTANTIATE_BoxObject(2, float, u32)
INSTANTIATE_BoxObject(3, float, u32)

// INSTANTIATE_Rectangle(2, double)
// INSTANTIATE_Rectangle(3, double)
} }
