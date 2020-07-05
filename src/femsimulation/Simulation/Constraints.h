/**
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

#pragma once

#include "Setup.h"

/****************************************************************************************************/
class Constraint {
public:
    enum Type {
        Attachment = 0,
        FEM
    };
    Constraint(Type type) : m_type(type) {}
    virtual ~Constraint() = default;
    Type type() const { return m_type; }
    virtual float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const        = 0;
    virtual void  getWLaplacianContribution(StdVT<Tripletf>& laplacian_1d_triplets) const = 0;
protected:
    Type m_type;
};

/****************************************************************************************************/
class AttachmentConstraint : public Constraint {
public:
    AttachmentConstraint(u32 vIdx, const Vec3f& fixedPos, float stiffness) :
        Constraint(Constraint::Type::Attachment), m_vIdx(vIdx), m_fixedPos(fixedPos), m_stiffness(stiffness) {}
    void setStiffness(float stiffness) { m_stiffness = stiffness; }
    virtual float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const;
    virtual void  getWLaplacianContribution(StdVT<Tripletf>& triplets) const;
private:
    u32   m_vIdx;
    Vec3f m_fixedPos;
    float m_stiffness { 0 };
};

/****************************************************************************************************/
class FEMConstraint : public Constraint {
public:
    enum Material {
        Corotational = 0,
        StVK,
        NeoHookeanExtendLog
    };
    FEMConstraint(const Vec4ui& vIDs, const VecXf& x);
    void                    setFEMMaterial(Material type, float mu, float lambda, float kappa);
    float                   getMassContribution(VecXf& m, VecXf& m_1d);
    std::pair<Mat3f, float> computeStressAndEnergyDensity(const Mat3f& F, bool computeEnergy = false) const;
    void                    computeLaplacianWeight();

    virtual float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const override;
    virtual void  getWLaplacianContribution(StdVT<Tripletf>& laplacian) const override;
private:
    Mat3f getMatrixDs(const VecXf& x) const;
    void  singularValueDecomp(Mat3f& U, Vec3f& SIGMA, Mat3f& V, const Mat3f& A, bool signed_svd = true) const;
    Material m_material;
    float    m_mu;
    float    m_lambda;
    float    m_kappa;
    static inline constexpr float m_neohookean_clamp_value { 0.1f };
public:                 /* public access for sag-free initializer */
    Vec4ui  m_vIDs;
    Mat3f   m_Dm;       /* [x0-x3|x1-x3|x2-x3]       */
    Mat3f   m_Dm_inv;   /* inverse of m_Dm           */
    Mat3f   m_Dm_inv_T; /* inverse transpose of m_Dm */
    Mat3x4f m_G;        /* Q = m_Dr^(-T) * IND;      */
    Mat4f   m_L;
    float   m_w;        /* 1/6 det(Dm);              */
};

#define LOGJ_QUADRATIC_EXTENSION /* comment this line to get linear extention of log(J) */
