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

#include <Attachment.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
void Attachment<DIM, Real_t>::evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& gradient) {
    gradient[_idx] += _stiffness * (x[_idx] - _fixed_pos);
}

template<int DIM, class Real_t>
Real_t Attachment<DIM, Real_t>::evaluateEnergy(const StdVT<Vec>& x) {
    return Real_t(0.5) * _stiffness * (x[_idx] - _fixed_pos).squaredNorm();
}

/****************************************************************************************************/
template class Attachment<2, float>;
template class Attachment<3, float>;
template class Attachment<2, double>;
template class Attachment<3, double>;
