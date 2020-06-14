#ifndef Magnum_Examples_FluidSimulation2D_FEMSolver_h
#define Magnum_Examples_FluidSimulation2D_FEMSolver_h
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

#include <Corrade/Containers/Pointer.h>

#include "FEMSolver/SolverData.h"

#include <Attachment.h>
#include <Element.h>

#include <memory>

namespace Magnum { namespace Examples {
/* 2D Affine Particle-in-Cell fluid solver */
class FEMSolver {
public:
    explicit FEMSolver();

    void advanceFrame(Float frameDuration);

private:
    /* Simulation */
    Float timestepCFL() const;
    void  moveParticles(Float dt);

    //    ParticleData _particles;

    std::size_t numVerts() const { return _vertPos.size(); }
    std::size_t numElements() const { return _elements.size(); }

    bool  performGradientDescentOneIteration(std::vector<Vector3>& x);
    void  evaluateGradient(const std::vector<Vector3>& x, std::vector<Vector3>& _gradient);
    Float evaluateEnergy(const std::vector<Vector3>& x);
    Float lineSearch(const std::vector<Vector3>& x, const std::vector<Vector3>& gradient_dir, const std::vector<Vector3>& descent_dir);

    void step();
    void draw();
    void reset() { _time = 0; _frame = 0; }

    ////////////////////////////////////////////////////////////////////////////////
    std::vector<Element>     _elements;
    std::vector<Vector3>     _vertPos;
    std::vector<UnsignedInt> _fixedVerts;
    std::vector<Attachment>  _attachmentConstr;
    std::vector<Vector3>     _gradient;
    std::vector<Vector3>     _externalForces;
    std::vector<Vector3>     _vertPredictedPos;
    std::vector<Vector3>     _oldVertPos;
    std::vector<Vector3>     _vertVel;

    /* Variable for line search */
    inline static constexpr Float _ls_alpha = Float(1.0);
    inline static constexpr Float _ls_beta  = Float(0.3);

    inline static constexpr Float _mass    = Float(1.0);
    inline static constexpr Float _massInv = Float(1.0);

    /* Frame counter */
    double      _time { 0 };
    std::size_t _frame { 0 };
};
} }

#endif
