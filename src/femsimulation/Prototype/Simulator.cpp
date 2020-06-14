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

#include <Magnum/GL/OpenGL.h>

#include <Simulator.h>
#include <SimParams.h>

//#include <InverseSolver/InverseElements.h>

#include <Common/ParallelHelpers/TaskScheduler.h>
#include <Common/ParallelHelpers/Scheduler.h>
#include <Common/ParallelHelpers/ParallelSTL.h>

#include <Animations/AnimationFactory.h>
#include <CollisionObjects/CollisionObjectFactory.h>
#include <Scenes/SceneFactory.h>

/****************************************************************************************************/
template<int DIM>
Simulator<DIM>::Simulator() {
    //TaskScheduler::setNumThreads(g_NumThreads);

    /* Setup scene  */
    const auto scene = SceneFactory<DIM, real, u32>::create();
    scene->setupScene(_vertPos, _fixedVerts, _elements);

    ////////////////////////////////////////////////////////////////////////////////
    /* Setup animation and collision objects */
    //    _animation = AnimationFactory<DIM, real>::create();
    //    auto collisionObj = CollisionObjectFactory<DIM, real, u32>::create();
    //    if(collisionObj != nullptr) {
    //        _CollisionObjs.emplace_back(std::move(collisionObj));
    //    }

    ////////////////////////////////////////////////////////////////////////////////
    /* Run inverse simulation */
    //    if(g_bInverseSimulation) {
    //        StdVT<std::pair<u32, Vec>> collisionInfo;
    //        for(auto& colObj : _CollisionObjs) {
    //            colObj->findCollision(_vertPos, collisionInfo);
    //        }
    //        InverseSolver::inverseElements<DIM, real, u32>(
    //            _vertPos, _fixedVerts,
    //            collisionInfo,
    //            _elements, g_gravity,
    //            g_ForceRegularization,
    //            g_TorqueRegularization,
    //            g_bCrashIfFailed,
    //            g_bPrintDebugDetails
    //            );
    //    }

    ////////////////////////////////////////////////////////////////////////////////
    /* Setup variables for integration */
    for(const auto fi : _fixedVerts) {
        _attachmentConstr.push_back(Attachment<DIM, real>(fi, _vertPos));
    }

    _gradient.resize(numVerts());
    _externalForces.assign(numVerts(), Vec::Zero());
    for(u64 v = 0; v < numVerts(); ++v) {
        _externalForces[v][1] = -g_gravity;
    }
    _vertVel.assign(numVerts(), Vec::Zero());
    _vertPredictedPos.assign(numVerts(), Vec::Zero());
}

/****************************************************************************************************/
template<int DIM>
void Simulator<DIM>::draw() {
    for(auto& e : _elements) {
        e.draw(_vertPos);
    }

    for(auto& colObj :_CollisionObjs) {
        colObj->draw();
    }

    /* Draw fixed vertices */
    //    glPointSize(10);
    //    glColor3f(1, 0, 0);
    //    glBegin(GL_POINTS);
    //    for(auto vi : _fixedVerts) {
    //        if constexpr (DIM == 2) {
    //            if constexpr (sizeof(real) == sizeof(float)) {
    //                glVertex2fv(reinterpret_cast<real*>(&_vertPos[vi]));
    //            } else {
    //                glVertex2dv(reinterpret_cast<real*>(&_vertPos[vi]));
    //            }
    //        } else {
    //            if constexpr (sizeof(real) == sizeof(float)) {
    //                glVertex3fv(reinterpret_cast<real*>(&_vertPos[vi]));
    //            } else {
    //                glVertex3dv(reinterpret_cast<real*>(&_vertPos[vi]));
    //            }
    //        }
    //    }
    //    glEnd();
}

/****************************************************************************************************/
template<int DIM>
void Simulator<DIM>::step() {
    /* Update system time and perform animation */
    _time += static_cast<double>(g_dt);
    _frame = static_cast<u64>(std::floor(_time * g_FPS));
    _animation->animate(_time, _vertPos, _attachmentConstr);

    ////////////////////////////////////////////////////////////////////////////////
    /* Resolve collision */
    for(auto& colObj: _CollisionObjs) {
        colObj->updateState(_time);
        colObj->resolveCollision(_vertPos, _vertVel, g_dt);
    }
    ////////////////////////////////////////////////////////////////////////////////
    /* Time integration */
    if(g_bIntegrationWithLineSearch) {
        const auto hSqr_mInv_over_2 = g_dt * g_dt * _massInv * real(0.5);
        _oldVertPos = _vertPos;
        Scheduler::parallel_for(
            numVerts(), [&](u64 vIdx) {
                _vertPredictedPos[vIdx] = _vertPos[vIdx] + _vertVel[vIdx] * g_dt +
                                          hSqr_mInv_over_2 * _externalForces[vIdx];
            });
        _vertPos = _vertPredictedPos;
        performGradientDescentOneIteration(_vertPos);

        /* Reset fixed positions */
        for(auto& fixedConstr : _attachmentConstr) {
            _vertPos[fixedConstr.getVertexIdx()] = fixedConstr.getFixedPosition();
        }

        Scheduler::parallel_for(
            numVerts(), [&](u64 vIdx) {
                Vec vel = 2 * (_vertPos[vIdx] - _oldVertPos[vIdx]) / g_dt -
                          _vertVel[vIdx];
                _vertVel[vIdx] = vel * (1 - g_damping);
            });
    } else { /* Explicit integration */
        for(auto& grad: _gradient) {
            grad.setZero();
        }

        /* Not thread-safe */
        for(u64 ei = 0; ei < numElements(); ++ei) {
            _elements[ei].evaluateGradient(_vertPos, _gradient);
        }

        Scheduler::parallel_for(numVerts(), [&](u64 vIdx) { _gradient[vIdx].y() += g_gravity; });

        for(const auto vi:_fixedVerts) {
            _gradient[vi].setZero();
        }

        Scheduler::parallel_for(numVerts(), [&](u64 vIdx) { _vertPos[vIdx] -= _gradient[vIdx] * g_dt; });
    }
}

/****************************************************************************************************/
template<int DIM>
bool Simulator<DIM>::performGradientDescentOneIteration(StdVT<Vec>& x) {
    static constexpr real eps = real(1e-6);
    evaluateGradient(x, _gradient);
    if(ParallelSTL::maxAbs<DIM, real>(_gradient) < eps) {
        return true;
    }

    // assign descent direction
    StdVT<Vec> descent_dir(_gradient.size());
    for(u64 i = 0; i < _gradient.size(); ++i) {
        descent_dir[i] = -_gradient[i];
    }

    // line search
    real step_size = lineSearch(x, _gradient, descent_dir);

    // update x
    Scheduler::parallel_for(numVerts(), [&](u64 vIdx) { x[vIdx] += descent_dir[vIdx] * step_size; });

    if(step_size < eps) {
        return true;
    } else {
        return false;
    }

    return false;
}

/****************************************************************************************************/
template<int DIM>
void Simulator<DIM>::evaluateGradient(const StdVT<Vec>& x, StdVT<Vec>& gradient) {
    gradient.assign(numVerts(), Vec::Zero());

    for(auto& e : _elements) {
        e.evaluateGradient(x, gradient);
    }
    for(auto att : _attachmentConstr) {
        att.evaluateGradient(x, gradient);
    }

    real h_square = g_dt * g_dt;
    Scheduler::parallel_for(numVerts(), [&](u64 vIdx) {
                                gradient[vIdx] = _mass * (x[vIdx] - _vertPredictedPos[vIdx]) +
                                                 0.5f * h_square * gradient[vIdx];
                            });
}

/****************************************************************************************************/
template<int DIM>
real Simulator<DIM>::evaluateEnergy(const StdVT<Vec>& x) {
    real inertia_term = 0;
    for(u64 i = 0; i < numVerts(); ++i) {
        inertia_term += real(0.5) * (x[i] - _vertPredictedPos[i]).squaredNorm() * _mass;
    }
    real h_square = g_dt * g_dt;

    real energy_pure_constraints = 0;
    for(auto ele : _elements) {
        energy_pure_constraints += ele.evaluateEnergy(x);
    }

    for(auto att : _attachmentConstr) {
        energy_pure_constraints += att.evaluateEnergy(x);
    }

    real energy = inertia_term + h_square * energy_pure_constraints;
    return energy;
}

/****************************************************************************************************/
#include <iostream>
template<int DIM>
real Simulator<DIM>::lineSearch(const StdVT<Vec>& x, const StdVT<Vec>& gradient_dir,
                                const StdVT<Vec>& descent_dir) {
    static constexpr real eps = real(1e-5);

    StdVT<Vec> x_plus_tdx(numVerts());
    real       t = 1.0f / _ls_beta;
    real       lhs, rhs;

    real currentObjectiveValue;
    try {
        currentObjectiveValue = evaluateEnergy(x);
    } catch(const std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    do {
        t *= _ls_beta;
        Scheduler::parallel_for(
            numVerts(), [&](u64 idx) {
                x_plus_tdx[idx] = x[idx] + t * descent_dir[idx];
            });

        lhs = 1e15f;
        rhs = 0;
        try {
            lhs = evaluateEnergy(x_plus_tdx);
        } catch(const std::exception&) {
            continue;
        }

        rhs = currentObjectiveValue;
        for(u64 i = 0; i < numVerts(); ++i) {
            rhs -= _ls_alpha * t * (gradient_dir[i].squaredNorm());
        }

        if(lhs >= rhs) {
            continue; // keep looping
        }
        break;        // exit looping
    } while (t > eps);

    if(t < eps) { t = 0; }
    return t;
}

/****************************************************************************************************/
template class Simulator<2>;
template class Simulator<3>;
