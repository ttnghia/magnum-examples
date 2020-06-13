#ifndef Magnum_Examples_ClothSimulation_MSSSolver_h
#define Magnum_Examples_ClothSimulation_MSSSolver_h
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

#include "Cloth.h"
#include "../fluidsimulation2d/FluidSolver/SolverData.h"

namespace Magnum { namespace Examples {
class MSSSolver {
public:
    MSSSolver() = default;

    /*  Simulation ops */
    void advanceFrame(Float frameDuration);

    /* Simulation parameter accessors */
    Float& stretchingStiffness() { return _stretchingStiffness; }
    Float& bendingStiffness() { return _bendingStiffness; }
    Float& constraintStiffness() { return _constraintStiffness; }
    Float& cflFactor() { return _cflFactor; }

    /* Data accessor */
    Cloth& getCloth() { return _cloth; }

private:
    Float timestepCFL() const;
    void  addGravity(Float dt);
    void  implicitIntegration(Float dt);
    void  updateVertexVelocities();
    void  updateVertexPositions(Float dt);

    /* Simulation data */
    Cloth              _cloth;
    LinearSystemSolver _linearSystemSolver;

    /* Simulation parameters */
    Float _stretchingStiffness{ 100.0f };
    Float _bendingStiffness{ 100.0f };
    Float _constraintStiffness{ 1000.0f };
    Float _cflFactor { 1.0f };
};
} }

#endif
