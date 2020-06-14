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

#include <Materials/CorotationalElasticity.h>

/****************************************************************************************************/
template<int DIM, class Real_t>
MatXxX<DIM, Real_t> CorotationalElasticity<DIM, Real_t>::computeStressTensor(const Mat& F) {
    const auto [U, SIGMA, V] = SVD(F);
    const Mat R = U * V.transpose();
    const Mat P = 2 * this->_mu * (F - R) + this->_lambda * ((R.transpose() * F).trace() - DIM) * R;
    return P;
}

/****************************************************************************************************/
template<int DIM, class Real_t>
Real_t CorotationalElasticity<DIM, Real_t>::computeEnergy(const Mat& F, const Real_t w) {
    const auto [U, SIGMA, V] = SVD(F);
    const Mat R = U * V.transpose();
    return w * (this->_mu * (F - R).squaredNorm() +
                Real_t(0.5) * this->_lambda * std::pow((R.transpose() * F).trace() - DIM, 2));
}

/****************************************************************************************************/
#include <Eigen/SVD>
template<int DIM, class Real_t>
std::tuple<MatXxX<DIM, Real_t>, VecX<DIM, Real_t>, MatXxX<DIM, Real_t>>
CorotationalElasticity<DIM, Real_t>::SVD(const Mat& A, bool signed_svd) {
    //Eigen Jacobi SVD
    Eigen::JacobiSVD<Mat> svd;
    svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto U     = svd.matrixU();
    auto V     = svd.matrixV();
    auto SIGMA = svd.singularValues();

    if(signed_svd) {
        Real_t detU = U.determinant();
        Real_t detV = V.determinant();
        if(detU < 0) {
            U.block<DIM, 1>(0, DIM - 1) *= -1;
            SIGMA[DIM - 1] *= -1;
        }
        if(detV < 0) {
            V.block<DIM, 1>(0, DIM - 1) *= -1;
            SIGMA[DIM - 1] *= -1;
        }
    }

    return { U, SIGMA, V };
}

/****************************************************************************************************/
/* Explicit instantiation */

#define INSTANTIATE_CorotationalElasticity(DIM, Real_t) template class CorotationalElasticity<DIM, Real_t>;

INSTANTIATE_CorotationalElasticity(2, float)
INSTANTIATE_CorotationalElasticity(3, float)

INSTANTIATE_CorotationalElasticity(2, double)
INSTANTIATE_CorotationalElasticity(3, double)
