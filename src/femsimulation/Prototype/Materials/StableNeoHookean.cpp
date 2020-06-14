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

namespace Magnum { namespace Examples {
/****************************************************************************************************/

Matrix3 StableNeoHookean::computeStressTensor(const Matrix3& F) {
    const Float Jminus1 = F.determinant() - Float(1.0) - this->_mu / this->_lambda;
    Matrix3     pJpF;

    pJpF[0] = Math::cross(F[1], F[2]);
    pJpF[1] = Math::cross(F[2], F[0]);
    pJpF[2] = Math::cross(F[0], F[1]);

    const Matrix3 P = this->_mu * F + this->_lambda * Jminus1 * pJpF;
    return P;
}

/****************************************************************************************************/

Float StableNeoHookean::computeEnergy(const Matrix3& F, const Float w) {
    const auto Jminus1 = F.determinant() - Float(1.0) - this->_mu / this->_lambda;
    Float      Ic{ 0 };
    for(UnsignedInt i = 0; i < 3; ++i) {
        for(UnsignedInt j = 0; j < 3; ++j) {
            Ic += F[i][j] * F[i][j];
        }
    }

    return w * (Float(0.5) * (this->_mu * (Ic - 3) + this->_lambda * Jminus1 * Jminus1));
}

/****************************************************************************************************/
/* Explicit instantiation */
} }
