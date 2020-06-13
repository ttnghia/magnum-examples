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

#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix3.h>

#include "ClothSolver.h"

namespace Magnum { namespace Examples {
void MSSSolver::advanceFrame(Float frameDuration) {
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

        addExternalForces(substep);
        implicitEulerIntegration(substep);
        updateVertexVelocities();
        updateVertexPositions(substep);
    }
}

Float MSSSolver::timestepCFL() const {
    Float maxVel = 0;
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            maxVel = Math::max(maxVel, Math::abs(_cloth.velocities[vidx].length()));
                        });
    return maxVel > 0 ? _cflFactor / maxVel : 1.0f;
}

void MSSSolver::addExternalForces(Float dt) {
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            if(!_cloth.isFixedVertex(vidx)) {
                                /* Gravity */
                                _cloth.velocities[vidx].y() -= Float(9.81) * dt;

                                /* Wind */
                            }
                        });
}

void MSSSolver::implicitEulerIntegration(Float dt) {
    _linearSystemSolver.resize(_cloth.getNumVertices() * 3);
    _linearSystemSolver.clear();

    const Float dtSqr = dt * dt;
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            if(!_cloth.isFixedVertex(vidx)) {
                                const Vector3 ppos = _cloth.positions[vidx];
                                const Vector3 pvel = _cloth.velocities[vidx];

                                Matrix3 sumLHSDx{ 0 };
                                Vector3 sumRHSDx{ 0 };
                                Vector3 springForce{ 0 };
                                Vector3 dampingForce{ 0 };

                                const auto& springList = _cloth.vertexSprings[vidx];
                                for(const Spring& spring : springList) {
                                    const UnsignedInt q = spring.targetVertex;
                                    Vector3 eij         = _cloth.positions[q] - ppos;
                                    const Float dij     = eij.length();
                                    if(dij < Float(1e-10)) {
                                        continue;
                                    }
                                    eij /= dij;

                                    Float springStiffness;
                                    if(spring.type == Spring::SpringType::Constraint) {
                                        springStiffness = _constraintStiffness;
                                    } else if(spring.type == Spring::SpringType::Stretching) {
                                        springStiffness = _stretchingStiffness;
                                    } else {
                                        springStiffness = _bendingStiffness;
                                    }
                                    springForce += (dij - spring.restLength) * eij * springStiffness;

                                    const Vector3 relVel = _cloth.velocities[q] - pvel;
                                    dampingForce        += Math::dot(eij, relVel) * eij * _damping;

                                    Matrix3 springDx;
                                    Matrix3 dampingDx;
                                    forceDerivative(eij, dij, spring.restLength,
                                                    springStiffness, _damping,
                                                    springDx, dampingDx);
                                    springDx  *= dtSqr;
                                    dampingDx *= dt;
                                    sumRHSDx  -= (springDx * relVel);

                                    springDx += dampingDx;
                                    sumLHSDx -= springDx;

                                    setMatrix(vidx, q, springDx);
                                }

                                sumLHSDx += Matrix3(1);
                                setMatrix(vidx, vidx, sumLHSDx);

                                sumRHSDx += (springForce + dampingForce) * dt;
                                for(UnsignedInt i = 0; i < 3; ++i) {
                                    _linearSystemSolver.rhs[vidx * 3] = sumRHSDx[i];
                                }
                            }
                        });

    _linearSystemSolver.solve();
}

void MSSSolver::updateVertexVelocities() {
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            if(!_cloth.isFixedVertex(vidx)) {
                                Vector3 dv;
                                for(UnsignedInt i = 0; i < 3; ++i) {
                                    dv[i] = static_cast<Float>(_linearSystemSolver.solution[vidx * 3 + i]);
                                }
                                _cloth.velocities[vidx] += dv;
                            }
                        });
}

void MSSSolver::updateVertexPositions(Float dt) {
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            if(!_cloth.isFixedVertex(vidx)) {
                                _cloth.positions[vidx] += _cloth.velocities[vidx] * dt;
                            }
                        });
}

/*  https://blog.mmacklin.com/2012/05/04/implicitsprings */
void MSSSolver::forceDerivative(const Vector3& eij, Float dij, Float d0,
                                Float stiffness, Float damping,
                                Matrix3& springDx, Matrix3& dampingDx) {
    Matrix3 xijxijT; /* = outerProduct(eij, eij) */
    for(UnsignedInt i = 0; i < 3; ++i) {
        for(UnsignedInt j = 0; j < 3; ++j) {
            xijxijT[i][j] = eij[i] * eij[j];
        }
    }
    springDx  = -stiffness * ((Float(1) - d0 / dij) * (Matrix3(1) - xijxijT) + xijxijT);
    dampingDx = -damping * xijxijT;
}

void MSSSolver::setMatrix(UnsignedInt p, UnsignedInt q, const Matrix3& mat) {
    for(UnsignedInt i = 0; i < 3; ++i) {
        for(UnsignedInt j = 0; j < 3; ++j) {
            _linearSystemSolver.matrix.addToElement(p * 3 + i,
                                                    q * 3 + j,
                                                    mat[i][j]);
        }
    }
}
} }
