#ifndef Magnum_Examples_FEMSimulationExample_Simulator_h
#define Magnum_Examples_FEMSimulationExample_Simulator_h
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

#include "Constraints.h"
#include "MathDefinitions.h"
#include "Mesh.h"

#include <deque>

namespace Magnum { namespace Examples {
class Simulator {
public:
    Simulator(TetMesh* mesh) : m_mesh(mesh) { CORRADE_INTERNAL_ASSERT(m_mesh != nullptr); }

    void reset();
    void advanceStep();
    void updateConstraintParameters();

private:
    void initConstraints();
    void calculateExternalForce(bool addWind = true);

    /* x is passed as the initial guess of the next postion (i.e. inertia term x = y = pos + vel*h + f_ext*h*h/m)
     * x will be changed during iteration
     * the final value of x will be the next position vector
     */
    bool  performLBFGSOneIteration(VecXf& x);
    Float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient);
    VecXf lbfgsKernelLinearSolve(const VecXf& gf_k);
    Float linesearch(const VecXf& x, Float energy, const VecXf& gradDir, const VecXf& descentDir,
                     Float& nextEnergy, VecXf& nextGradDir);
    void prefactorize();

public: /* public accessible parameters */
    struct {
        Vec3f gravity = Vec3f(0, -10, 0);
        Float damping { 0.002f };
        Float attachmentStiffness{ 1000.0f };

        Float dt { 1.0f / 30.0f };
        int   subSteps { 5 };
        Float time { 0 };
    } m_generalParams;

    struct {
        bool  enable     = false;
        Float timeEnable = 10; /* Disable if set to a negative number */
        Float magnitude  = 15;
        Float frequency  = 1;
    } m_wind;

    struct {
        int   type   = FEMConstraint::Material::NeoHookeanExtendLog;
        Float mu     = 40;
        Float lambda = 20;
        Float kappa  = 0; /* only for StVK material */
    } m_FEMMaterial;

private: /* simulation variables */
    TetMesh* m_mesh;
    Containers::Array<Constraint*> m_constraints;

    struct {
        VecXf y;
        VecXf externalForces;
    } m_integration;

    struct {
        const Float       epsLARGE      = 1e-4f;
        const Float       epsSMALL      = 1e-12f;
        const std::size_t historyLength = 5;

        Eigen::SimplicialLLT<SparseMatrixf, Eigen::Upper> lltSolver;

        bool              prefactored = false;
        VecXf             lastX;
        VecXf             lastGradient;
        std::deque<VecXf> yQueue;
        std::deque<VecXf> sQueue;
        MatX3f            linearSolveRhsN3;
    } m_lbfgs;

    struct {
        const UnsignedInt iterations { 10 };
        const Float       alpha { 0.03f };
        const Float       beta{ 0.5f };

        bool  firstIteration;
        Float prefetchedEnergy;
        VecXf prefetchedGradient;
    } m_lineSearch;
};
} }

#endif
