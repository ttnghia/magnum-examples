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

#include "Simulation/Constraints.h"
#include "Simulation/Mesh.h"
#include "Simulation/MathDefinitions.h"

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
    float evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient);
    VecXf lbfgsKernelLinearSolve(const VecXf& gf_k);
    float linesearch(const VecXf& x, float energy, const VecXf& gradDir, const VecXf& descentDir,
                     float& nextEnergy, VecXf& nextGradDir);
    void prefactorize();

public: /* public accessible parameters */
    struct {
        Vec3f gravity = Vec3f(0, -10, 0);
        float damping { 0.002f };
        float attachmentStiffness{ 1000.0f };

        float dt { 1.0f / 30.0f };
        int   subSteps { 5 };
        float time { 0 };
    } m_generalParams;

    struct {
        bool  enable     = false;
        float timeEnable = 10; /* Disable if set to a negative number */
        float magnitude  = 15;
        float frequency  = 1;
    } m_wind;

    struct {
        int   type   = FEMConstraint::Material::NeoHookeanExtendLog;
        float mu     = 40;
        float lambda = 20;
        float kappa  = 0; /* only for StVK material */
    } m_FEMMaterial;

private: /* simulation variables */
    TetMesh*           m_mesh;
    StdVT<Constraint*> m_constraints;

    struct {
        VecXf y;
        VecXf externalForces;
    } m_integration;

    struct {
        const float epsLARGE      = 1e-4f;
        const float epsSMALL      = 1e-12f;
        const u64   historyLength = 5;

        Eigen::SimplicialLLT<SparseMatrixf, Eigen::Upper> lltSolver;

        bool              prefactored = false;
        VecXf             lastX;
        VecXf             lastGradient;
        std::deque<VecXf> yQueue;
        std::deque<VecXf> sQueue;
        MatX3f            linearSolveRhsN3;
    } m_lbfgs;

    struct {
        const u32   iterations { 10 };
        const float alpha { 0.03f };
        const float beta{ 0.5f };

        bool  firstIteration;
        float prefetchedEnergy;
        VecXf prefetchedGradient;
    } m_lineSearch;
};
} }
