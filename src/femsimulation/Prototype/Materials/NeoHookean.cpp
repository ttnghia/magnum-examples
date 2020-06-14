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

#include <Materials/NeoHookean.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> NeoHookean<DIM, Real_t>::computeStressTensor(const Mat& F) {
    Real_t J = F.determinant();
    if(J <= 0) {
        J = Real_t(1e-20);
    }
    const Mat FinvT = F.inverse().transpose();
    const Mat P     = this->_mu * (F - FinvT) + this->_lambda * std::log(J) * FinvT;
    return P;
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_NeoHookean(DIM, Real_t) template class NeoHookean<DIM, Real_t>;

INSTANTIATE_NeoHookean(2, float)
INSTANTIATE_NeoHookean(3, float)

INSTANTIATE_NeoHookean(2, double)
INSTANTIATE_NeoHookean(3, double)
