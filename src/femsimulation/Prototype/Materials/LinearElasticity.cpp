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

#include <Materials/LinearElasticity.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> LinearElasticity<DIM, Real_t>::computeStressTensor(const Mat& F) {
    const Mat P = this->_mu * (F + F.transpose() - 2 * Mat::Identity()) +
                  this->_lambda * (F - Mat::Identity()).trace() * Mat::Identity();
    return P;
}

/****************************************************************************************************/
template<int DIM, class Real_t>
Real_t LinearElasticity<DIM, Real_t>::computeEnergy(const Mat& F, const Real_t w) {
    const Mat E = Real_t(0.5) * (F + F.transpose()) - Mat::Identity();
    return w * (this->_mu * E.squaredNorm() + Real_t(0.5) * this->_lambda * std::pow(E.trace(), 2));
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_LinearElasticity(DIM, Real_t) template class LinearElasticity<DIM, Real_t>;

INSTANTIATE_LinearElasticity(2, float)
INSTANTIATE_LinearElasticity(3, float)

INSTANTIATE_LinearElasticity(2, double)
INSTANTIATE_LinearElasticity(3, double)
