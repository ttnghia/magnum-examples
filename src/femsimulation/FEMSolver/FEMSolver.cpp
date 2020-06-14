/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vla3ír Vondruš <mosra@centrum.cz>
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

#include "FEMSolver/FEMSolver.h"

#include <random>

#include <SimParams.h>

//#include <InverseSolver/InverseElements.h>

#include <Scenes/SceneFactory.h>

namespace Magnum { namespace Examples {
template<class IndexType, class Function>
void parallel_for(IndexType endIdx, Function&& function) {
    for(IndexType i = 0; i < endIdx; ++i) {
        function(i);
    }
}

void FEMSolver::advanceFrame(Float frameDuration) {
    Float frameTime = 0;

    while(frameTime < frameDuration) {
        Float       substep       = timestepCFL();
        const Float remainingTime = frameDuration - frameTime;
        if(frameTime + substep > frameDuration) {
            substep = remainingTime;
        } else if(frameTime + Float(1.5) * substep > frameDuration) {
            substep = remainingTime * Float(0.5);
        }
        frameTime += substep;

        /* Advect particles */
        moveParticles(substep);
    }
}

Float FEMSolver::timestepCFL() const {
    Float maxVel = 0;
    //    _grid.u.loop1D([&](std::std::size_t i) {
    //                       maxVel = Math::max(maxVel, Math::abs(_grid.u.data()[i]));
    //                   });

    return 0;
}

void FEMSolver::moveParticles(Float dt) {
    // _particles.loopAll([&](UnsignedInt p) {
    //                        const Vector2 newPos    = _particles.positions[p] + _particles.velocities[p] * dt;
    //                        _particles.positions[p] = _grid.constrainBoundary(newPos);
    //                    });
}

/****************************************************************************************************/

FEMSolver::FEMSolver() {
    //TaskScheduler::setNumThreads(g_NumThreads);

    /* Setup scene  */
    const auto scene = SceneFactory::create();
    scene->setupScene(_vertPos, _fixedVerts, _elements);

    ////////////////////////////////////////////////////////////////////////////////
    /* Setup animation and collision objects */
    //    _animation = AnimationFactory::create();
    //    auto collisionObj = CollisionObjectFactory<3, Float, UnsignedInt>::create();
    //    if(collisionObj != nullptr) {
    //        _CollisionObjs.emplace_back(std::move(collisionObj));
    //    }

    ////////////////////////////////////////////////////////////////////////////////
    /* Run inverse simulation */
    //    if(g_bInverseSimulation) {
    //        std::vector<std::pair<UnsignedInt, Vector3>> collisionInfo;
    //        for(auto& colObj : _CollisionObjs) {
    //            colObj->findCollision(_vertPos, collisionInfo);
    //        }
    //        InverseSolver::inverseElements<3, Float, UnsignedInt>(
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
        _attachmentConstr.push_back(Attachment(fi, _vertPos));
    }

    _gradient.resize(numVerts());
    _externalForces.assign(numVerts(), Vector3 { 0 });
    for(std::size_t v = 0; v < numVerts(); ++v) {
        _externalForces[v][1] = -g_gravity;
    }
    _vertVel.assign(numVerts(), Vector3 { 0 });
    _vertPredictedPos.assign(numVerts(), Vector3 { 0 });
}

/****************************************************************************************************/

void FEMSolver::draw() {
    for(auto& e : _elements) {
        e.draw(_vertPos);
    }

    /* Draw fixed vertices */
    //    glPointSize(10);
    //    glColor3f(1, 0, 0);
    //    glBegin(GL_POINTS);
    //    for(auto vi : _fixedVerts) {
    //        if constexpr (3 == 2) {
    //            if constexpr (sizeof(Float) == sizeof(float)) {
    //                glVertex2fv(reinterpret_cast<Float*>(&_vertPos[vi]));
    //            } else {
    //                glVertex2dv(reinterpret_cast<Float*>(&_vertPos[vi]));
    //            }
    //        } else {
    //            if constexpr (sizeof(Float) == sizeof(float)) {
    //                glVertex3fv(reinterpret_cast<Float*>(&_vertPos[vi]));
    //            } else {
    //                glVertex3dv(reinterpret_cast<Float*>(&_vertPos[vi]));
    //            }
    //        }
    //    }
    //    glEnd();
}

/****************************************************************************************************/

void FEMSolver::step() {
    /* Update system time and perform animation */
    _time += static_cast<double>(g_dt);
    _frame = static_cast<std::size_t>(std::floor(_time * g_FPS));
    //_animation->animate(_time, _vertPos, _attachmentConstr);

    ////////////////////////////////////////////////////////////////////////////////
    /* Resolve collision */
    //    for(auto& colObj: _CollisionObjs) {
    // colObj->updateState(_time);
    // colObj->resolveCollision(_vertPos, _vertVel, g_dt);
    //    }
    ////////////////////////////////////////////////////////////////////////////////
    /* Time integration */
    if(g_bIntegrationWithLineSearch) {
        const auto hSqr_mInv_over_2 = g_dt * g_dt * _massInv * Float(0.5);
        _oldVertPos = _vertPos;
        parallel_for(
            numVerts(), [&](std::size_t vIdx) {
                _vertPredictedPos[vIdx] = _vertPos[vIdx] + _vertVel[vIdx] * g_dt +
                                          hSqr_mInv_over_2 * _externalForces[vIdx];
            });
        _vertPos = _vertPredictedPos;
        performGradientDescentOneIteration(_vertPos);

        /* Reset fixed positions */
        for(auto& fixedConstr : _attachmentConstr) {
            _vertPos[fixedConstr.getVertexIdx()] = fixedConstr.getFixedPosition();
        }

        parallel_for(
            numVerts(), [&](std::size_t vIdx) {
                Vector3 vel = 2 * (_vertPos[vIdx] - _oldVertPos[vIdx]) / g_dt -
                              _vertVel[vIdx];
                _vertVel[vIdx] = vel * (1 - g_damping);
            });
    } else { /* Explicit integration */
        for(auto& grad: _gradient) {
            grad = Vector3{ 0 };
        }

        /* Not thread-safe */
        for(std::size_t ei = 0; ei < numElements(); ++ei) {
            _elements[ei].evaluateGradient(_vertPos, _gradient);
        }

        parallel_for(numVerts(), [&](std::size_t vIdx) { _gradient[vIdx].y() += g_gravity; });

        for(const auto vi:_fixedVerts) {
            _gradient[vi] = Vector3{ 0 };
        }

        parallel_for(numVerts(), [&](std::size_t vIdx) { _vertPos[vIdx] -= _gradient[vIdx] * g_dt; });
    }
}

/****************************************************************************************************/

bool FEMSolver::performGradientDescentOneIteration(std::vector<Vector3>& x) {
    static constexpr Float eps = Float(1e-6);
    evaluateGradient(x, _gradient);

    Float maxAbs{ 0 };
    for(const Vector3& grad: _gradient) {
        for(UnsignedInt i = 0; i < 3; ++i) {
            if(std::abs(grad[i]) < eps) {
                return true;
            }
        }
    }

    // assign descent direction
    std::vector<Vector3> descent_dir(_gradient.size());
    for(std::size_t i = 0; i < _gradient.size(); ++i) {
        descent_dir[i] = -_gradient[i];
    }

    // line search
    Float step_size = lineSearch(x, _gradient, descent_dir);

    // update x
    parallel_for(numVerts(), [&](std::size_t vIdx) { x[vIdx] += descent_dir[vIdx] * step_size; });

    if(step_size < eps) {
        return true;
    } else {
        return false;
    }

    return false;
}

/****************************************************************************************************/

void FEMSolver::evaluateGradient(const std::vector<Vector3>& x, std::vector<Vector3>& gradient) {
    gradient.assign(numVerts(), Vector3 { 0 });

    for(auto& e : _elements) {
        e.evaluateGradient(x, gradient);
    }
    for(auto att : _attachmentConstr) {
        att.evaluateGradient(x, gradient);
    }

    Float h_square = g_dt * g_dt;
    parallel_for(numVerts(), [&](std::size_t vIdx) {
                     gradient[vIdx] = _mass * (x[vIdx] - _vertPredictedPos[vIdx]) +
                                      0.5f * h_square * gradient[vIdx];
                 });
}

/****************************************************************************************************/

Float FEMSolver::evaluateEnergy(const std::vector<Vector3>& x) {
    Float inertia_term = 0;
    for(std::size_t i = 0; i < numVerts(); ++i) {
        const Float d = (x[i] - _vertPredictedPos[i]).length();
        inertia_term += Float(0.5) * d * d * _mass;
    }
    Float h_square = g_dt * g_dt;

    Float energy_pure_constraints = 0;
    for(auto ele : _elements) {
        energy_pure_constraints += ele.evaluateEnergy(x);
    }

    for(auto att : _attachmentConstr) {
        energy_pure_constraints += att.evaluateEnergy(x);
    }

    Float energy = inertia_term + h_square * energy_pure_constraints;
    return energy;
}

/****************************************************************************************************/

Float FEMSolver::lineSearch(const std::vector<Vector3>& x, const std::vector<Vector3>& gradient_dir,
                            const std::vector<Vector3>& descent_dir) {
    static constexpr Float eps = Float(1e-5);

    std::vector<Vector3> x_plus_tdx(numVerts());
    Float                t = 1.0f / _ls_beta;
    Float                lhs, rhs;

    Float currentObjectiveValue;
    try {
        currentObjectiveValue = evaluateEnergy(x);
    } catch(const std::exception& e) {
        // std::cout << e.what() << std::endl;
    }

    do {
        t *= _ls_beta;
        parallel_for(
            numVerts(), [&](std::size_t idx) {
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
        for(std::size_t i = 0; i < numVerts(); ++i) {
            const Float d = gradient_dir[i].length();
            rhs -= _ls_alpha * t * d * d;
        }

        if(lhs >= rhs) {
            continue; // keep looping
        }
        break;        // exit looping
    } while (t > eps);

    if(t < eps) { t = 0; }
    return t;
}
} }
