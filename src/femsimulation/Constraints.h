#ifndef Magnum_Examples_FEMSimulationExample_Constraint_h
#define Magnum_Examples_FEMSimulationExample_Constraint_h
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

#include "MathDefinitions.h"

namespace Magnum { namespace Examples {
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

class FEMConstraint : public Constraint {
public:
    enum Material {
        Corotational = 0,
        StVK,
        NeoHookeanExtendLog
    };
    FEMConstraint(const Vec4ui& vIDs, const VecXf& x);
    void  setFEMMaterial(Material type, float mu, float lambda, float kappa);
    float getMassContribution(VecXf& m, VecXf& m_1d);
    float computeStressAndEnergyDensity(const Mat3f& F, Mat3f& P) const;
    void  computeLaplacianWeight();

    virtual float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) const override;
    virtual void  getWLaplacianContribution(StdVT<Tripletf>& laplacian) const override;
private:
    Mat3f getMatrixDs(const VecXf& x) const;
    void  singularValueDecomp(Mat3f& U, Vec3f& SIGMA, Mat3f& V, const Mat3f& A, bool signed_svd = true) const;
    Material    m_material;
    float       m_mu;
    float       m_lambda;
    float       m_kappa;
    const float m_neohookean_clamp_value { 0.1f };
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
} }

#endif
