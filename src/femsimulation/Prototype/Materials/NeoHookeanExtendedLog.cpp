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

#include <Materials/NeoHookeanExtendedLog.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> NeoHookeanExtendedLog<DIM, Real_t>::computeStressTensor(const Mat& F) {
    const Real_t     J     = F.determinant();
    const Mat        FinvT = F.inverse().transpose();
    constexpr Real_t J0    = m_NeoHookeanClampValue;

    Mat P = this->_mu * F;
    if(J > J0) {
        const auto logJ = std::log(J);
        P += (-this->_mu) * FinvT + this->_lambda * logJ * FinvT;
    } else {
        const auto fJ    = std::log(J0) + (J - J0) / J0;
        const auto dfJdJ = Real_t(1.0) / J0;
        P += ((-this->_mu) + this->_lambda * fJ) * dfJdJ * J * FinvT;
    }
    return P;
}

/****************************************************************************************************/
template<int DIM, class Real_t>
Real_t NeoHookeanExtendedLog<DIM, Real_t>::computeEnergy(const Mat& F, const Real_t w) {
    constexpr Real_t J0     = m_NeoHookeanClampValue;
    Mat              FtF    = F.transpose() * F;
    const auto       I1     = FtF.trace();
    const auto       J      = F.determinant();
    auto             energy = Real_t(0.5) * this->_mu * (I1 - DIM);
    if(J > J0) {
        const auto logJ = std::log(J);
        energy += (-this->_mu) * logJ + Real_t(0.5) * this->_lambda * logJ * logJ;
    } else {
        const auto fJ = std::log(J0) + (J - J0) / J0;
        energy += (-this->_mu) * fJ + Real_t(0.5) * this->_lambda * fJ * fJ;
    }

    return w * energy;
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_NeoHookeanExtendedLog(DIM, Real_t) template class NeoHookeanExtendedLog<DIM, Real_t>;

INSTANTIATE_NeoHookeanExtendedLog(2, float)
INSTANTIATE_NeoHookeanExtendedLog(3, float)

INSTANTIATE_NeoHookeanExtendedLog(2, double)
INSTANTIATE_NeoHookeanExtendedLog(3, double)
