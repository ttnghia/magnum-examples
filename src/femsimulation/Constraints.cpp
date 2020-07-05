/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>
        2020 — Nghia Truong <nghiatruong.vn@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Corrade/Containers/GrowableArray.h>

#include "Constraints.h"
#include <Eigen/SVD>

namespace Magnum { namespace Examples {
Float AttachmentConstraint::evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const {
    const Vec3f d = x.block3(m_vIdx) - m_fixedPos;
    const Vec3f g = m_stiffness * d;
    gradient.block3(m_vIdx) += g;
    const Float energy = 0.5f * m_stiffness * d.squaredNorm();
    return energy;
}

void AttachmentConstraint::getWLaplacianContribution(Containers::Array<Tripletf>& triplets) const {
    arrayAppend(triplets, Containers::InPlaceInit, Tripletf(m_vIdx, m_vIdx, m_stiffness));
}

FEMConstraint::FEMConstraint(const Vector4ui& vIDs, const VecXf& x) :
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

void FEMConstraint::setFEMMaterial(Material type, Float mu, Float lambda, Float kappa) {
    m_material = type;
    m_mu       = mu;
    m_lambda   = lambda;
    m_kappa    = kappa;

    /* Compute the Laplacian weight which depends on the material parameters */
    computeLaplacianWeight();
}

void FEMConstraint::computeLaplacianWeight() {
    Float laplacianCoeff{ 0 };
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

Float FEMConstraint::getMassContribution(VecXf& m, VecXf& m_1d) {
    const Float vw = 0.25f * m_w;
    for(UnsignedInt i = 0; i != 4; i++) {
        m[3 * m_vIDs[i] + 0] += vw;
        m[3 * m_vIDs[i] + 1] += vw;
        m[3 * m_vIDs[i] + 2] += vw;
        m_1d[m_vIDs[i]]      += vw;
    }
    return m_w;
}

Float FEMConstraint::computeStressAndEnergyDensity(const Mat3f& F, Mat3f& P) const {
    Float e_density = 0;
    switch(m_material) {
        case Material::Corotational: {
            Mat3f U;
            Mat3f V;
            Vec3f SIGMA;
            singularValueDecomp(U, SIGMA, V, F);
            const Mat3f R          = U * V.transpose();
            const Mat3f FmR        = F - R;
            const Float traceRTFm3 = (R.transpose() * F).trace() - 3;
            P         = 2 * m_mu * FmR + m_lambda * traceRTFm3 * R;
            e_density = m_mu * FmR.squaredNorm() + 0.5f * m_lambda * traceRTFm3 * traceRTFm3;
        }
        break;
        case Material::StVK: {
            const Mat3f I      = Mat3f::Identity();
            const Mat3f E      = 0.5f * (F.transpose() * F - I);
            const Float traceE = E.trace();
            P = F * (2 * m_mu * E + m_lambda * traceE * I);
            const Float J = F.determinant();
            e_density = m_mu * E.squaredNorm() + 0.5f * m_lambda * traceE * traceE;
            if(J < 1 && J != 0) {
                const Float one_mJd6 = (1 - J) / 6;
                P         += -m_kappa / 24 * one_mJd6 * one_mJd6 * J * F.inverse().transpose();
                e_density += m_kappa / 12 * one_mJd6 * one_mJd6 * one_mJd6;
            }
        }
        break;
        case Material::NeoHookeanExtendLog: {
            P         = m_mu * F;
            e_density = 0.5f * m_mu * ((F.transpose() * F).trace() - 3);

            const Float J = F.determinant();
            if(J == 0) { /* degenerated tet */
                break;
            }
            const Float J0    = m_neohookean_clamp_value;
            const Mat3f FinvT = F.inverse().transpose(); /* F is invertible if J != 0 */
            if(J > J0) {
                const Float logJ = std::log(J);
                P         += (-m_mu + m_lambda * logJ) * FinvT;
                e_density += (-m_mu + 0.5f * m_lambda * logJ) * logJ;
            } else {
                const Float JmJ0dJ0 = (J - J0) / J0;
#ifdef LOGJ_QUADRATIC_EXTENSION
                const Float fJ    = std::log(J0) + JmJ0dJ0 - 0.5f * JmJ0dJ0 * JmJ0dJ0;
                const Float dfJdJ = (1.0f - JmJ0dJ0) / J0;
#else
                Float fJ    = std::log(J0) + JmJ0dJ0;
                Float dfJdJ = 1.0f / J0;
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

Float FEMConstraint::evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const {
    Mat3f       P;
    const Mat3f F         = getMatrixDs(x) * m_Dm_inv;
    const Float e_density = computeStressAndEnergyDensity(F, P);
    const Mat3f H         = m_w * P * m_Dm_inv_T;
    Vec3f       H3        = Vec3f::Zero();
    for(UnsignedInt i = 0; i < 3; i++) {
        gradient.block3(m_vIDs[i]) += H.col(i);
        H3 += H.col(i);
    }
    gradient.block3(m_vIDs[3]) -= H3;
    return e_density * m_w;
}

void FEMConstraint::getWLaplacianContribution(Containers::Array<Tripletf>& triplets) const {
    for(UnsignedInt i = 0; i < 4; i++) {
        for(UnsignedInt j = 0; j < 4; j++) {
            arrayAppend(triplets, Containers::InPlaceInit, Tripletf(m_vIDs[i], m_vIDs[j], m_L(i, j)));
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
        Float detU = U.determinant();
        Float detV = V.determinant();
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
