/**;
 * Copyright 2020 Nghia Truong <nghiatruong.vn@gmail.com>
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

#include "Constraints.h"
#include <Eigen/SVD>

namespace Magnum { namespace Examples {
float AttachmentConstraint::evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const {
    const Vec3f d = x.block3(m_vIdx) - m_fixedPos;
    const Vec3f g = m_stiffness * d;
    gradient.block3(m_vIdx) += g;
    const float energy = 0.5f * m_stiffness * d.squaredNorm();
    return energy;
}

void AttachmentConstraint::getWLaplacianContribution(StdVT<Tripletf>& triplets) const {
    triplets.push_back(Tripletf(m_vIdx, m_vIdx, m_stiffness));
}

FEMConstraint::FEMConstraint(const Vec4ui& vIDs, const VecXf& x) :
    Constraint(Constraint::Type::FEM), m_vIDs(vIDs) {
    for(size_t i = 0; i < 3; ++i) {
        m_Dm.col(i) = x.block3(m_vIDs[i]) - x.block3(m_vIDs[3]);
    }

    m_Dm_inv   = m_Dm.inverse();
    m_Dm_inv_T = m_Dm_inv.transpose();
    m_w        = std::abs(m_Dm.determinant()) / 6.0f;

    Mat3x4f IND;
    IND.block<3, 3>(0, 0) = Mat3f::Identity();
    IND.block<3, 1>(0, 3) = Vec3f(-1, -1, -1);
    m_G = m_Dm_inv_T * IND;
}

void FEMConstraint::setFEMMaterial(Material type, float mu, float lambda, float kappa) {
    m_material = type;
    m_mu       = mu;
    m_lambda   = lambda;
    m_kappa    = kappa;

    /* Compute the Laplacian weight which depends on the material parameters */
    computeLaplacianWeight();
}

void FEMConstraint::computeLaplacianWeight() {
    float laplacianCoeff{ 0 };
    switch(m_material) {
        case Material::Corotational:
            // 2mu (x-1) + lambda (x-1)
            laplacianCoeff = 2 * m_mu + m_lambda;
            break;
        case Material::StVK:
            // mu * (x^2  - 1) + 0.5lambda * (x^3 - x)
            // 10% window
            laplacianCoeff = 2 * m_mu + 1.0033 * m_lambda;
            //// 20% window
            //m_laplacian_coeff = 2 * m_mu + 1.0126 * m_lambda;
            break;
        case Material::NeoHookeanExtendLog:
            // mu * x - mu / x + lambda * log(x) / x
            // 10% window
            laplacianCoeff = 2.0066 * m_mu + 1.0122 * m_lambda;
            //// 20% window
            //m_laplacian_coeff = 2.0260 * m_mu + 1.0480 * m_lambda;
            break;
        default:
            break;
    }
    m_L = laplacianCoeff * m_w * m_G.transpose() * m_G;
}

float FEMConstraint::getMassContribution(VecXf& m, VecXf& m_1d) {
    const float vw = 0.25f * m_w;
    for(u32 i = 0; i != 4; i++) {
        m[3 * m_vIDs[i] + 0] += vw;
        m[3 * m_vIDs[i] + 1] += vw;
        m[3 * m_vIDs[i] + 2] += vw;
        m_1d[m_vIDs[i]]      += vw;
    }
    return m_w;
}

float FEMConstraint::computeStressAndEnergyDensity(const Mat3f& F, Mat3f& P) const {
    float e_density = 0;
    switch(m_material) {
        case Material::Corotational: {
            Mat3f U;
            Mat3f V;
            Vec3f SIGMA;
            singularValueDecomp(U, SIGMA, V, F);
            const Mat3f R          = U * V.transpose();
            const Mat3f FmR        = F - R;
            const float traceRTFm3 = (R.transpose() * F).trace() - 3;
            P         = 2 * m_mu * FmR + m_lambda * traceRTFm3 * R;
            e_density = m_mu * FmR.squaredNorm() + 0.5f * m_lambda * traceRTFm3 * traceRTFm3;
        }
        break;
        case Material::StVK: {
            const Mat3f I      = Mat3f::Identity();
            const Mat3f E      = 0.5f * (F.transpose() * F - I);
            const float traceE = E.trace();
            P = F * (2 * m_mu * E + m_lambda * traceE * I);
            const float J = F.determinant();
            e_density = m_mu * E.squaredNorm() + 0.5f * m_lambda * traceE * traceE;
            if(J < 1 && J != 0) {
                const float one_mJd6 = (1 - J) / 6;
                P         += -m_kappa / 24 * one_mJd6 * one_mJd6 * J * F.inverse().transpose();
                e_density += m_kappa / 12 * one_mJd6 * one_mJd6 * one_mJd6;
            }
        }
        break;
        case Material::NeoHookeanExtendLog: {
            P         = m_mu * F;
            e_density = 0.5f * m_mu * ((F.transpose() * F).trace() - 3);

            const float J = F.determinant();
            if(J == 0) { /* degenerated tet */
                break;
            }
            const float J0    = m_neohookean_clamp_value;
            const Mat3f FinvT = F.inverse().transpose(); /* F is invertible if J != 0 */
            if(J > J0) {
                const float logJ = std::log(J);
                P         += (-m_mu + m_lambda * logJ) * FinvT;
                e_density += (-m_mu + 0.5f * m_lambda * logJ) * logJ;
            } else {
                const float JmJ0dJ0 = (J - J0) / J0;
#ifdef LOGJ_QUADRATIC_EXTENSION
                const float fJ    = std::log(J0) + JmJ0dJ0 - 0.5f * JmJ0dJ0 * JmJ0dJ0;
                const float dfJdJ = (1.0f - JmJ0dJ0) / J0;
#else
                float fJ    = std::log(J0) + JmJ0dJ0;
                float dfJdJ = 1.0f / J0;
#endif
                P         += (-m_mu + m_lambda * fJ) * dfJdJ * J * FinvT;
                e_density += (-m_mu + 0.5f * m_lambda * fJ) * fJ;
            }
        }
        break;
        default:
            break;
    }
    return e_density;
}

float FEMConstraint::evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const {
    Mat3f       P;
    const Mat3f F         = getMatrixDs(x) * m_Dm_inv;
    const float e_density = computeStressAndEnergyDensity(F, P);
    const Mat3f H         = m_w * P * m_Dm_inv_T;
    Vec3f       H3        = Vec3f::Zero();
    for(u32 i = 0; i < 3; i++) {
        gradient.block3(m_vIDs[i]) += H.col(i);
        H3 += H.col(i);
    }
    gradient.block3(m_vIDs[3]) -= H3;
    return e_density * m_w;
}

void FEMConstraint::getWLaplacianContribution(StdVT<Tripletf>& triplets) const {
    for(u32 i = 0; i < 4; i++) {
        for(u32 j = 0; j < 4; j++) {
            triplets.emplace_back(Tripletf(m_vIDs[i], m_vIDs[j], m_L(i, j)));
        }
    }
}

Mat3f FEMConstraint::getMatrixDs(const VecXf& x) const {
    Mat3f Ds;
    for(size_t i = 0; i < 3; ++i) {
        Ds.col(i) = x.block3(m_vIDs[i]) - x.block3(m_vIDs[3]);
    }
    return Ds;
}

void FEMConstraint::singularValueDecomp(Mat3f& U, Vec3f& SIGMA, Mat3f& V, const Mat3f& A, bool signed_svd) const {
    Eigen::JacobiSVD<Mat3f> svd;
    svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U     = svd.matrixU();
    V     = svd.matrixV();
    SIGMA = svd.singularValues();
    if(signed_svd) {
        float detU = U.determinant();
        float detV = V.determinant();
        if(detU < 0) {
            U.block<3, 1>(0, 2) *= -1;
            SIGMA[2] *= -1;
        }
        if(detV < 0) {
            V.block<3, 1>(0, 2) *= -1;
            SIGMA[2] *= -1;
        }
    }
}
} }
