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

#include <Corrade/Containers/Array.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector4.h>

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
    virtual Float evaluateEnergyAndGradient(const EgVecXf& x, EgVecXf& gradient) const = 0;
    virtual void  getWLaplacianContribution(Containers::Array<EgTripletf>& laplacian_1d_triplets) const = 0;
protected:
    Type m_type;
};

class AttachmentConstraint : public Constraint {
public:
    AttachmentConstraint(UnsignedInt vIdx, const EgVec3f& fixedPos, Float stiffness) :
        Constraint(Constraint::Type::Attachment), m_vIdx(vIdx), m_fixedPos(fixedPos), m_stiffness(stiffness) {}
    void setStiffness(Float stiffness) { m_stiffness = stiffness; }
    virtual Float evaluateEnergyAndGradient(const EgVecXf& x, EgVecXf& gradient) const;
    virtual void  getWLaplacianContribution(Containers::Array<EgTripletf>& triplets) const;
private:
    UnsignedInt m_vIdx;
    EgVec3f       m_fixedPos;
    Float       m_stiffness { 0 };
};

class FEMConstraint : public Constraint {
public:
    enum Material {
        Corotational = 0,
        StVK,
        NeoHookeanExtendLog
    };
    FEMConstraint(const Vector4ui& vIDs, const EgVecXf& x);
    void  setFEMMaterial(Material type, Float mu, Float lambda, Float kappa);
    Float getMassContribution(EgVecXf& m, EgVecXf& m_1d);
    Float computeStressAndEnergyDensity(const EgMat3f& F, EgMat3f& P) const;
    void  computeLaplacianWeight();

    virtual Float evaluateEnergyAndGradient(const EgVecXf& x, EgVecXf& gradient) const override;
    virtual void  getWLaplacianContribution(Containers::Array<EgTripletf>& laplacian) const override;
private:
    EgMat3f getMatrixDs(const EgVecXf& x) const;
    void  singularValueDecomp(EgMat3f& U, EgVec3f& SIGMA, EgMat3f& V, const EgMat3f& A, bool signed_svd = true) const;
    Material    m_material;
    Float       m_mu;
    Float       m_lambda;
    Float       m_kappa;
    const Float m_neohookean_clamp_value { 0.1f };
public:                   /* public access for sag-free initializer */
    Vector4ui m_vIDs;
    EgMat3f     m_Dm;       /* [x0-x3|x1-x3|x2-x3]       */
    EgMat3f     m_Dm_inv;   /* inverse of m_Dm           */
    EgMat3f     m_Dm_inv_T; /* inverse transpose of m_Dm */
    EgMat3x4f   m_G;        /* Q = m_Dr^(-T) * IND;      */
    EgMat4f     m_L;
    Float     m_w;        /* 1/6 det(Dm);              */
};

#define LOGJ_QUADRATIC_EXTENSION /* comment this line to get linear extention of log(J) */
} }

#endif
