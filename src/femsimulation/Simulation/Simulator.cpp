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

#include "Simulator.h"
#include "SagFreeInitializer/TetrahedraInitializer.h"

/****************************************************************************************************/
void Simulator::reset() {
    /* Reset timing */
    m_generalParams.time = 0;

    /* Reset wind */
    m_wind.enable = false;

    /* Reset mesh data */
    m_mesh->reset();
}

/****************************************************************************************************/
void Simulator::initializeSagFreeSimulation() {
    /* Must clear and re-initialize constraints to ensure we're working on fresh data */
    for(auto c : m_constraints) {
        delete c;
    }
    m_constraints.resize(0);
    initConstraints();
    CORRADE_INTERNAL_ASSERT(m_constraints.size() > 0);

    /* Extract FEM constraints */
    StdVT<FEMConstraint*> femConstraints;
    for(const auto constraint : m_constraints) {
        if(constraint->type() == Constraint::Type::FEM) {
            const auto tet = static_cast<FEMConstraint*>(constraint);
            CORRADE_INTERNAL_ASSERT(tet != nullptr);
            femConstraints.push_back(tet);
        }
    }

    /* Compute gravity vector */
    calculateExternalForce(false);

    /* The list of fixed vertices must be sorted beforehand */
    std::sort(m_mesh->m_fixedVerts.begin(), m_mesh->m_fixedVerts.end());

    /* Call the initializer */
    SagFreeInitializer::initializeSagFree(m_mesh->m_positions, m_mesh->m_fixedVerts,
                                          m_integration.externalForces,
                                          femConstraints,
                                          m_sagfreeInit.regularization,
                                          m_sagfreeInit.softConstraintStiffness,
                                          m_sagfreeInit.cgTolerance,
                                          m_sagfreeInit.cgMaxIters);

    /* Need to prefactor the Laplacean, as tetrahedra have changed */
    m_lbfgs.prefactored = false;
}

/****************************************************************************************************/
void Simulator::advanceStep() {
    if(m_constraints.size() == 0) {
        initConstraints();
        CORRADE_INTERNAL_ASSERT(m_constraints.size() > 0);
    }
    /* Enable wind if applicable */
    if(m_wind.timeEnable >= 0 && m_generalParams.time >= m_wind.timeEnable && !m_wind.enable) {
        m_wind.enable = true;
    }
    calculateExternalForce(m_wind.enable);

    /* Change global time step, as it will be used somewhere else */
    const float old_dt = m_generalParams.dt;
    m_generalParams.dt = m_generalParams.dt / m_generalParams.subSteps;

    for(int step = 0; step < m_generalParams.subSteps; ++step) {
        m_integration.y = m_mesh->m_positions +
                          m_mesh->m_velocities * m_generalParams.dt +
                          m_mesh->m_invMassMatrix * m_integration.externalForces * m_generalParams.dt * m_generalParams.dt;

        VecXf x = m_integration.y; /* x will be modified during line search */
        m_lineSearch.firstIteration = true;
        bool converge = false;
        for(u32 iter = 0; !converge && iter < m_lineSearch.iterations; ++iter) {
            converge = performLBFGSOneIteration(x);
            m_lineSearch.firstIteration = false;
        }

        m_mesh->m_velocities = (x - m_mesh->m_positions) / m_generalParams.dt;
        m_mesh->m_positions  = x;
        if(std::abs(m_generalParams.damping) > 0) {
            m_mesh->m_velocities *= 1 - m_generalParams.damping;
        }
    }
    m_generalParams.dt    = old_dt;
    m_generalParams.time += m_generalParams.dt;
}

/****************************************************************************************************/
void Simulator::updateConstraintParameters() {
    for(const auto c: m_constraints) {
        if(c->type() == Constraint::Type::FEM) {
            static_cast<FEMConstraint*>(c)->setFEMMaterial(
                static_cast<FEMConstraint::Material>(m_FEMMaterial.type),
                m_FEMMaterial.mu, m_FEMMaterial.lambda, m_FEMMaterial.kappa);
        } else if(c->type() == Constraint::Type::Attachment) {
            static_cast<AttachmentConstraint*>(c)->setStiffness(m_generalParams.attachmentStiffness);
        }
    }

    /* Need to prerefactor the Laplacean, as material has changed */
    m_lbfgs.prefactored = false;
}

/****************************************************************************************************/
void Simulator::initConstraints() {
    /* Setup attachment constraints */
    for(const auto vIdx : m_mesh->m_fixedVerts) {
        const Vec3f fixedPoint = m_mesh->m_positions.block3(vIdx);
        m_constraints.push_back(new AttachmentConstraint(vIdx, fixedPoint, m_generalParams.attachmentStiffness));
    }

    /* Compute mass for the vertices */
    float total_volume = 0;
    m_mesh->m_massMatrix.resize(3 * m_mesh->m_numVerts);
    m_mesh->m_massMatrix1D.resize(m_mesh->m_numVerts);
    m_mesh->m_invMassMatrix.resize(3 * m_mesh->m_numVerts);
    m_mesh->m_massMatrix.setZero();
    m_mesh->m_massMatrix1D.setZero();
    m_mesh->m_invMassMatrix.setZero();

    for(const auto& tet:  m_mesh->m_tets) {
        FEMConstraint* c = new FEMConstraint(tet, m_mesh->m_positions_t0);
        c->setFEMMaterial(static_cast<FEMConstraint::Material>(m_FEMMaterial.type),
                          m_FEMMaterial.mu, m_FEMMaterial.lambda, m_FEMMaterial.kappa);
        total_volume += c->getMassContribution(m_mesh->m_massMatrix.diagonal(),
                                               m_mesh->m_massMatrix1D.diagonal());
        m_constraints.push_back(c);
    }
    m_mesh->m_massMatrix   = m_mesh->m_massMatrix * (m_mesh->m_totalMass / total_volume);
    m_mesh->m_massMatrix1D = m_mesh->m_massMatrix1D * (m_mesh->m_totalMass / total_volume);

    /* Compute inverse mass for the vertices */
    m_mesh->m_invMassMatrix.resize(3 * m_mesh->m_numVerts);
    for(u32 i = 0; i < 3 * m_mesh->m_numVerts; ++i) {
        m_mesh->m_invMassMatrix.diagonal()[i] = 1.0f / m_mesh->m_massMatrix.diagonal()[i];
    }
}

/****************************************************************************************************/
void Simulator::calculateExternalForce(bool addWind) {
    Vec3f force = m_generalParams.gravity;
    if(addWind) {
        force.z() = (std::sin((m_generalParams.time - m_wind.timeEnable) * m_wind.frequency) + 0.5f) * m_wind.magnitude;
    }
    m_integration.externalForces.resize(m_mesh->m_numVerts * 3);
    for(u32 i = 0; i < m_mesh->m_numVerts; ++i) {
        m_integration.externalForces.block3(i) = force;
    }
    m_integration.externalForces = m_mesh->m_massMatrix * m_integration.externalForces;
}

/****************************************************************************************************/
bool Simulator::performLBFGSOneIteration(VecXf& x) {
    static constexpr float historyLength = m_lbfgs.historyLength;
    static constexpr float epsLARGE      = m_lbfgs.epsLARGE;
    static constexpr u64   epsSMALL      = m_lbfgs.epsSMALL;

    bool  converged = false;
    float currentEnergy;
    VecXf currentGrad;
    auto& yQueue             = m_lbfgs.yQueue;
    auto& sQueue             = m_lbfgs.sQueue;
    auto& prefetchedEnergy   = m_lineSearch.prefetchedEnergy;
    auto& prefetchedGradient = m_lineSearch.prefetchedGradient;

    if(m_lineSearch.firstIteration) {
        yQueue.clear();
        sQueue.clear();
        prefactorize();

        currentEnergy        = evaluateEnergyAndGradient(x, currentGrad);
        m_lbfgs.lastX        = x;
        m_lbfgs.lastGradient = currentGrad;

        VecXf p_k = -lbfgsKernelLinearSolve(currentGrad);
        if(-p_k.dot(currentGrad) < epsSMALL || p_k.norm() / x.norm() < epsLARGE) {
            converged = true;
        }
        const float alpha_k = linesearch(x, currentEnergy, currentGrad, p_k,
                                         prefetchedEnergy, prefetchedGradient);
        x += alpha_k * p_k;
    } else {
        currentEnergy = prefetchedEnergy;
        currentGrad   = prefetchedGradient;

        const VecXf s_k = x - m_lbfgs.lastX;
        const VecXf y_k = currentGrad - m_lbfgs.lastGradient;
        if(sQueue.size() > m_lbfgs.historyLength) {
            sQueue.pop_back();
            yQueue.pop_back();
        }
        sQueue.push_front(std::move(s_k));
        yQueue.push_front(std::move(y_k));

        m_lbfgs.lastX        = x;
        m_lbfgs.lastGradient = currentGrad;
        VecXf q = currentGrad;

        StdVT<float> rho;
        StdVT<float> alpha;
        const u64    queueSize  = sQueue.size();
        const u64    visitBound = (historyLength < queueSize) ? historyLength : queueSize;
        for(u64 i = 0; i < visitBound; ++i) {
            const float yi_dot_si = yQueue[i].dot(sQueue[i]);
            if(yi_dot_si < m_lbfgs.epsSMALL) {
                return true;
            }
            const float rho_i = 1.0f / yi_dot_si;
            rho.push_back(rho_i);
            alpha.push_back(rho[i] * sQueue[i].dot(q));
            q = q - alpha[i] * yQueue[i];
        }

        VecXf p_k = -lbfgsKernelLinearSolve(q);
        for(int i = int(visitBound) - 1; i >= 0; --i) {
            const float beta = rho[i] * yQueue[i].dot(p_k);
            p_k -= sQueue[i] * (alpha[i] - beta);
        }
        if(-p_k.dot(currentGrad) < epsSMALL || p_k.squaredNorm() < epsSMALL) {
            converged = true;
        }
        const float alpha_k = linesearch(x, currentEnergy, currentGrad, p_k,
                                         prefetchedEnergy, prefetchedGradient);
        x += alpha_k * p_k;
    }
    return converged;
}

/****************************************************************************************************/
float Simulator::evaluateEnergyAndGradient(const VecXf& x, VecXf& gradient) {
    const float hSqr          = m_generalParams.dt * m_generalParams.dt;
    const VecXf xmy           = x - m_integration.y;
    const VecXf Mxmy          = m_mesh->m_massMatrix * xmy;
    const float energyInertia = 0.5f * xmy.transpose() * Mxmy;
    gradient.resize(m_mesh->m_numVerts * 3);
    gradient.setZero();
    float energyConstraints = 0;
    for(const auto c: m_constraints) {
        energyConstraints += c->evaluateEnergyAndGradient(x, gradient);
    }
    gradient = Mxmy + hSqr * gradient;
    const float energy = energyInertia + hSqr * energyConstraints;
    return energy;
}

/****************************************************************************************************/
VecXf Simulator::lbfgsKernelLinearSolve(const VecXf& rhs) {
    auto& rhsN3 = m_lbfgs.linearSolveRhsN3;
    rhsN3.resize(rhs.size() / 3, 3);
    std::memcpy(rhsN3.data(), rhs.data(), sizeof(float) * rhs.size());
    const MatX3f r_n3 = m_lbfgs.lltSolver.solve(rhsN3);

    VecXf r;
    r.resize(rhs.size());
    std::memcpy(r.data(), r_n3.data(), sizeof(float) * r.size());
    return r;
}

/****************************************************************************************************/
float Simulator::linesearch(const VecXf& x, float energy, const VecXf& gradDir,
                            const VecXf& descentDir, float& nextEnergy, VecXf& nextGradDir) {
    VecXf x_plus_tdx(m_mesh->m_numVerts * 3);
    float t      = 1.0f / m_lineSearch.beta;
    float objVal = energy;
    do{
        t         *= m_lineSearch.beta;
        x_plus_tdx = x + t * descentDir;
        const float lhs = evaluateEnergyAndGradient(x_plus_tdx, nextGradDir);
        const float rhs = objVal + m_lineSearch.alpha * t * gradDir.transpose().dot(descentDir);
        if(lhs < rhs) {
            nextEnergy = lhs;
            break;
        }
    } while (t > 1e-5f);

    if(t < 1e-5f) {
        t           = 0;
        nextEnergy  = energy;
        nextGradDir = gradDir;
    }
    return t;
}

/****************************************************************************************************/
void Simulator::prefactorize() {
    if(!m_lbfgs.prefactored) {
        /* Compute the Laplacian matrix */
        StdVT<Tripletf> triplets;
        for(const auto c: m_constraints) {
            c->getWLaplacianContribution(triplets);
        }
        SparseMatrixf weightedLaplacian1D;
        weightedLaplacian1D.resize(m_mesh->m_numVerts, m_mesh->m_numVerts);
        weightedLaplacian1D.setFromTriplets(triplets.begin(), triplets.end());
        weightedLaplacian1D *= (m_generalParams.dt * m_generalParams.dt);
        for(u32 i = 0; i < weightedLaplacian1D.rows(); ++i) {
            weightedLaplacian1D.coeffRef(i, i) += m_mesh->m_massMatrix1D.diagonal()[i];
        }

        /* Prefactor the matrix */
        SparseMatrixf& A = weightedLaplacian1D;
        m_lbfgs.lltSolver.analyzePattern(A);
        m_lbfgs.lltSolver.factorize(A);
        float regularization = 1e-10f;
        while(m_lbfgs.lltSolver.info() != Eigen::Success) {
            regularization *= 10;
            for(int i = 0; i < A.rows(); ++i) {
                A.coeffRef(i, i) += regularization;
            }
            m_lbfgs.lltSolver.factorize(A);
        }
        m_lbfgs.prefactored = true;
    }
}
