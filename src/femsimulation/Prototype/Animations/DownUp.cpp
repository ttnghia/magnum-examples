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

#include <Animations/DownUp.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
void DownUp<DIM, Real_t>::animate(const double time, StdVT<Vec>& vertexPos,
                                  StdVT<Attachment<DIM, Real_t>>& attachmentConstr) {
    if(!g_bAnimation) {
        return;
    }

    auto lerp = [] (Real_t a, Real_t b, Real_t t) {
                    return (1 - t) * a + t * b;
                };
    auto easing = [] (Real_t t) {
                      return t * t;
                  };

    if(time >= double(g_AnimationStartTime) && (_Y0.size() == 0 || _Y1.size() == 0)) {
        for(const auto& fv: attachmentConstr) {
            const auto vidx = fv.getVertexIdx();
            _Y0[vidx] = vertexPos[vidx][1];
            _Y1[vidx] = vertexPos[vidx][1] - Real_t(5);
        }
    }

    static Real_t t = 0;
    if(time >= double(g_AnimationStartTime) &&
       time <= double(g_AnimationStartTime) + 2 * double(g_AnimationDuration)) {
        static Real_t step = Real_t(1) / Real_t(g_AnimationDuration * 1000);
        if(t > 1 || t < 0) {
            step *= -Real_t(1.0);
        }
        t += step;
        Real_t easing_t = easing(t);

        for(auto& fv : attachmentConstr) {
            const auto vidx = fv.getVertexIdx();
            vertexPos[vidx][1] = lerp(_Y0[vidx], _Y1[vidx], easing_t);
            fv.setFixedPosition(vertexPos[vidx]);
        }
    }
}

/****************************************************************************************************/
/* Explicit instantiation */
#define INSTANTIATE_DownUp(DIM, Real_t) template class DownUp<DIM, Real_t>;

INSTANTIATE_DownUp(2, float)
INSTANTIATE_DownUp(3, float)

INSTANTIATE_DownUp(2, double)
INSTANTIATE_DownUp(3, double)
