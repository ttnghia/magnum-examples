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

#include <Materials/StableNeoHookean.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> StableNeoHookean<DIM, Real_t>::computeStressTensor(const Mat& F) {
    const Real_t Jminus1 = F.determinant() - Real_t(1.0) - this->_mu / this->_lambda;
    Mat          pJpF;
    if constexpr (DIM == 2) {
        pJpF(0, 0) = F(1, 1);
        pJpF(0, 1) = -F(1, 0);
        pJpF(1, 0) = -F(0, 1);
        pJpF(1, 1) = F(0, 0);
    } else {
        pJpF.col(0) = F.col(1).cross(F.col(2));
        pJpF.col(1) = F.col(2).cross(F.col(0));
        pJpF.col(2) = F.col(0).cross(F.col(1));
    }

    const Mat P = this->_mu * F + this->_lambda * Jminus1 * pJpF;
    return P;
}

/****************************************************************************************************/
template<int DIM, class Real_t>
Real_t StableNeoHookean<DIM, Real_t>::computeEnergy(const Mat& F, const Real_t w) {
    const auto Jminus1 = F.determinant() - Real_t(1.0) - this->_mu / this->_lambda;
    const auto Ic      = F.squaredNorm();
    return w * (Real_t(0.5) * (this->_mu * (Ic - DIM) + this->_lambda * Jminus1 * Jminus1));
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_StableNeoHookean(DIM, Real_t) template class StableNeoHookean<DIM, Real_t>;

INSTANTIATE_StableNeoHookean(2,  float)
INSTANTIATE_StableNeoHookean(3,  float)

INSTANTIATE_StableNeoHookean(2, double)
INSTANTIATE_StableNeoHookean(3, double)
