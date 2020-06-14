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

#include <Magnum/GL/OpenGL.h>

#include <Element.h>
//#include <Tensor.h>
#include <Materials/MaterialFactory.h>

#include <iostream>
namespace Magnum { namespace Examples {
/****************************************************************************************************/

Element::Element(const std::vector<UnsignedInt> idx, const std::vector<Vector3>& x) {
    _idx = std::move(idx);

    _Dm = getMatrixDs(x);
    updateElementWeight();

    _Dm_inv  = _Dm.inverted();
    _Dm_invT = _Dm_inv.transposed();

    _material = MaterialFactory::create();
}

/****************************************************************************************************/

Matrix3 Element::getMatrixDs(const std::vector<Vector3>& x) {
    Matrix3 Ds;
    for(int d = 0; d < 3; ++d) {
        Ds[d] = x[_idx[d]] - x[_idx[3]];
    }
    return Ds;
}

/****************************************************************************************************/

Matrix3 Element::getDeformationGradient(const std::vector<Vector3>& x) {
    const Matrix3 Ds = getMatrixDs(x);
    const Matrix3 F  = Ds * _Dm_inv;
    return F;
}

/****************************************************************************************************/

void Element::evaluateGradient(const std::vector<Vector3>& x, std::vector<Vector3>& gradient) {
    const Matrix3 F = getDeformationGradient(x);

    Matrix3 P = _material->computeStressTensor(F);
    Matrix3 H = _w * P * _Dm_inv.transposed();

    Vector3 fLast = Vector3{ 0 };
    for(int d = 0; d < 3; ++d) {
        fLast -= H[d];
    }

    for(UnsignedInt i = 0; i < 3; i++) {
        gradient[_idx[i]] += H[i];
    }
    gradient[_idx[3]] += fLast;
}

/****************************************************************************************************/

Float Element::evaluateEnergy(const std::vector<Vector3>& x) {
    const Matrix3 F = getDeformationGradient(x);
    return _material->computeEnergy(F, _w);
}

/****************************************************************************************************/

/****************************************************************************************************/

void Element::updateElementWeight() {
    const auto detDm = std::abs(_Dm.determinant());
    _w = (3 == 2) ?
         detDm * Float(1.0 / 2.0) :
         detDm* Float(1.0 / 6.0);
}

/****************************************************************************************************/

void Element::draw(std::vector<Vector3>& p) {
    //    glColor3f(0, 0, 1);
    //    glBegin(GL_LINES);
    //    if constexpr (3 == 2) {
    //        if constexpr (sizeof(Float) == sizeof(float)) {
    //            glVertex2fv(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex2fv(reinterpret_cast<Float*>(&p[_idx[1]]));
    //            glVertex2fv(reinterpret_cast<Float*>(&p[_idx[1]])); glVertex2fv(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex2fv(reinterpret_cast<Float*>(&p[_idx[2]])); glVertex2fv(reinterpret_cast<Float*>(&p[_idx[0]]));
    //        } else {
    //            glVertex2fd(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex2fd(reinterpret_cast<Float*>(&p[_idx[1]]));
    //            glVertex2fd(reinterpret_cast<Float*>(&p[_idx[1]])); glVertex2fd(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex2fd(reinterpret_cast<Float*>(&p[_idx[2]])); glVertex2fd(reinterpret_cast<Float*>(&p[_idx[0]]));
    //        }
    //    } else {
    //        if constexpr (sizeof(Float) == sizeof(float)) {
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[1]]));
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[3]]));
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[1]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[2]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[3]]));
    //            glVertex3fv(reinterpret_cast<Float*>(&p[_idx[3]])); glVertex3fv(reinterpret_cast<Float*>(&p[_idx[1]]));
    //        } else {
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[1]]));
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[3]]));
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[1]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[2]]));
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[2]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[3]]));
    //            glVertex3fd(reinterpret_cast<Float*>(&p[_idx[3]])); glVertex3fd(reinterpret_cast<Float*>(&p[_idx[1]]));
    //        }
    //    }
    //    glEnd();
}

/****************************************************************************************************/
} }
