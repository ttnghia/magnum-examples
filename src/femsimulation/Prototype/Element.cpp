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
#include <Tensor.h>
#include <Materials/MaterialFactory.h>

#include <iostream>

/****************************************************************************************************/
template<int DIM>
Element<DIM>::Element(const StdVT<u32> idx, const StdVT<Vec>& x) {
    _idx = std::move(idx);

    _Dm = getMatrixDs(x);
    updateElementWeight();

    _Dm_inv  = _Dm.inverse();
    _Dm_invT = _Dm_inv.transpose();

    _material = MaterialFactory<DIM, real>::create();
}

/****************************************************************************************************/
template<int DIM>
MatXxX<DIM, real> Element<DIM>::getMatrixDs(const StdVT<Vec>& x) {
    Mat Ds;
    for(int d = 0; d < DIM; ++d) {
        Ds.col(d) = x[_idx[d]] - x[_idx[DIM]];
    }
    return Ds;
}

/****************************************************************************************************/
template<int DIM>
MatXxX<DIM, real> Element<DIM>::getDeformationGradient(const StdVT<Vec>& x) {
    const Mat Ds = getMatrixDs(x);
    const Mat F  = Ds * _Dm_inv;
    return F;
}

/****************************************************************************************************/
template<int DIM>
void Element<DIM>::evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& gradient) {
    const Mat F = getDeformationGradient(x);

    Mat P = _material->computeStressTensor(F);
    Mat H = _w * P * _Dm_inv.transpose();

    Vec fLast = Vec::Zero();
    for(int d = 0; d < DIM; ++d) {
        fLast -= H.col(d);
    }

    for(u32 i = 0; i < DIM; i++) {
        gradient[_idx[i]] += H.col(i);
    }
    gradient[_idx[DIM]] += fLast;
}

/****************************************************************************************************/
template<int DIM>
real Element<DIM>::evaluateEnergy(const StdVT<Vec>& x) {
    const Mat F = getDeformationGradient(x);
    return _material->computeEnergy(F, _w);
}

/****************************************************************************************************/
template<int DIM>
real Element<DIM>::getTorqueMagnitude(const Mat& H) const {
    if constexpr (DIM == 2) {
        const auto f0 = H.col(0);
        const auto f1 = H.col(1);
        const auto r0 = _Dm.col(0);
        const auto r1 = _Dm.col(1);

        const auto f0x = f0[0];
        const auto f0y = f0[1];
        const auto f1x = f1[0];
        const auto f1y = f1[1];
        const auto r0x = r0[0];
        const auto r0y = r0[1];
        const auto r1x = r1[0];
        const auto r1y = r1[1];

        /* Torque in 2D = cross(f, r) = f_x * r_y - f_y * r_x
         * Where r[ev] = vertex[ev] - vertex[DIM] = Dm.col(ev)
         */
        return std::abs((f0x * r0y - f0y * r0x) + (f1x * r1y - f1y * r1x));
    } else {
        const auto f0 = H.col(0);
        const auto f1 = H.col(1);
        const auto f2 = H.col(2);
        const auto r0 = _Dm.col(0);
        const auto r1 = _Dm.col(1);
        const auto r2 = _Dm.col(2);

        Vec torque;

        /* torque in x = t_x = f_y * r_z - f_z * r_y */
        torque[0] = f0[1] * r0[2] - f0[2] * r0[1] +
                    f1[1] * r1[2] - f1[2] * r1[1] +
                    f2[1] * r2[2] - f2[2] * r2[1];

        /* torque in y = t_y = f_z * r_x - f_x * r_z */
        torque[1] = f0[2] * r0[0] - f0[0] * r0[2] +
                    f1[2] * r1[0] - f1[0] * r1[2] +
                    f2[2] * r2[0] - f2[0] * r2[2];

        /* torque in z = t_z = f_x * r_y - f_y * r_x */
        torque[2] = f0[0] * r0[1] - f0[1] * r0[0] +
                    f1[0] * r1[1] - f1[1] * r1[0] +
                    f2[0] * r2[1] - f2[1] * r2[0];

        return torque.norm();
    }
}

/****************************************************************************************************/
template<int DIM>
void Element<DIM>::updateElementWeight() {
    const auto detDm = std::abs(_Dm.determinant());
    _w = (DIM == 2) ?
         detDm * real(1.0 / 2.0) :
         detDm* real(1.0 / 6.0);
}

/****************************************************************************************************/
template<int DIM>
void Element<DIM>::draw(StdVT<Vec>& p) {
    //    glColor3f(0, 0, 1);
    //    glBegin(GL_LINES);
    //    if constexpr (DIM == 2) {
    //        if constexpr (sizeof(real) == sizeof(float)) {
    //            glVertex2fv(reinterpret_cast<real*>(&p[_idx[0]])); glVertex2fv(reinterpret_cast<real*>(&p[_idx[1]]));
    //            glVertex2fv(reinterpret_cast<real*>(&p[_idx[1]])); glVertex2fv(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex2fv(reinterpret_cast<real*>(&p[_idx[2]])); glVertex2fv(reinterpret_cast<real*>(&p[_idx[0]]));
    //        } else {
    //            glVertex2fd(reinterpret_cast<real*>(&p[_idx[0]])); glVertex2fd(reinterpret_cast<real*>(&p[_idx[1]]));
    //            glVertex2fd(reinterpret_cast<real*>(&p[_idx[1]])); glVertex2fd(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex2fd(reinterpret_cast<real*>(&p[_idx[2]])); glVertex2fd(reinterpret_cast<real*>(&p[_idx[0]]));
    //        }
    //    } else {
    //        if constexpr (sizeof(real) == sizeof(float)) {
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[1]]));
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[3]]));
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[1]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[2]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[3]]));
    //            glVertex3fv(reinterpret_cast<real*>(&p[_idx[3]])); glVertex3fv(reinterpret_cast<real*>(&p[_idx[1]]));
    //        } else {
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[1]]));
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[0]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[3]]));
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[1]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[2]]));
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[2]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[3]]));
    //            glVertex3fd(reinterpret_cast<real*>(&p[_idx[3]])); glVertex3fd(reinterpret_cast<real*>(&p[_idx[1]]));
    //        }
    //    }
    //    glEnd();
}

/****************************************************************************************************/
template class Element<2>;
template class Element<3>;
