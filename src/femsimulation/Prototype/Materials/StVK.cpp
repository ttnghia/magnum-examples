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

#include <Materials/StVK.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> StVK<DIM, Real_t>::computeStressTensor(const Mat& F) {
    const Mat    I = Mat::Identity();
    const Mat    E = Real_t(0.5) * (F.transpose() * F - I);
    Mat          P = F * (2 * this->_mu * E + this->_lambda * E.trace() * I);
    const Real_t J = F.determinant();
    if(J < 1) {
        if constexpr (DIM == 2) {
            P += -_kappa / 6 * std::pow((1 - J), 2) * J * F.inverse().transpose();
        } else {
            P += -_kappa / 24 * std::pow((1 - J) / 6, 2) * J * F.inverse().transpose();
        }
    }
    return P;
}

/****************************************************************************************************/
template<int DIM, class Real_t>
Real_t StVK<DIM, Real_t>::computeEnergy(const Mat& F, const Real_t w) {
    const Mat I      = Mat::Identity();
    const Mat E      = Real_t(0.5) * (F.transpose() * F - I);
    auto      energy = this->_mu * E.squaredNorm() + Real_t(0.5) * this->_lambda * std::pow(E.trace(), 2);

    const Real_t J = F.determinant();
    if(J < 1) {
        if constexpr (DIM == 2) {
            energy += _kappa / 6 * std::pow((1 - J) / 2, 2);
        } else {
            energy += _kappa / 12 * std::pow((1 - J) / 6, 3);
        }
    }

    return w * energy;
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_StVK(DIM, Real_t) template class StVK<DIM, Real_t>;

INSTANTIATE_StVK(2, float)
INSTANTIATE_StVK(3, float)

INSTANTIATE_StVK(2, double)
INSTANTIATE_StVK(3, double)
