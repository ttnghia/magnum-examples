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

namespace Magnum { namespace Examples {
/****************************************************************************************************/

Matrix3 NeoHookean::computeStressTensor(const Matrix3& F) {
    Float J = F.determinant();
    if(J <= 0) {
        J = Float(1e-20);
    }
    const Matrix3 FinvT = F.inverted().transposed();
    const Matrix3 P     = this->_mu * (F - FinvT) + this->_lambda * std::log(J) * FinvT;
    return P;
}

/****************************************************************************************************/
/* Explicit instantiation */
} }
