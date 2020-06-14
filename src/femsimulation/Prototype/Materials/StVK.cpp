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

namespace Magnum { namespace Examples {
/****************************************************************************************************/

Matrix3 StVK::computeStressTensor(const Matrix3& F) {
    const Matrix3 I = Matrix3{ 1 };
    const Matrix3 E = Float(0.5) * (F.transposed() * F - I);
    Matrix3       P = F * (2 * this->_mu * E + this->_lambda * E.trace() * I);
    const Float   J = F.determinant();
    if(J < 1) {
        P += -_kappa / 24 * std::pow((1 - J) / 6, 2) * J * F.inverted().transposed();
    }
    return P;
}

/****************************************************************************************************/

Float StVK::computeEnergy(const Matrix3& F, const Float w) {
    const Matrix3 I = Matrix3{ 1 };
    const Matrix3 E = Float(0.5) * (F.transposed() * F - I);

    Float energy{ 0 };
    for(UnsignedInt i = 0; i < 3; ++i) {
        for(UnsignedInt j = 0; j < 3; ++j) {
            energy += E[i][j] * E[i][j];
        }
    }
    energy *= this->_mu;
    energy += Float(0.5) * this->_lambda * std::pow(E.trace(), 2);

    const Float J = F.determinant();
    if(J < 1) {
        energy += _kappa / 12 * std::pow((1 - J) / 6, 3);
    }

    return w * energy;
}

/****************************************************************************************************/
/* Explicit instantiation */
} }
