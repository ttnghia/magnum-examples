#ifndef Magnum_Examples_FluidSimulation2D_SolverData_h
#define Magnum_Examples_FluidSimulation2D_SolverData_h
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

#include <vector>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix.h>
#include <Magnum/Math/Vector2.h>

namespace Magnum { namespace Examples {
struct ParticleData {
    explicit ParticleData(Float cellSize) : particleRadius{cellSize* 0.5f} {}

    UnsignedInt size() const { return static_cast<UnsignedInt>(positions.size()); }

    void addParticles(const std::vector<Vector2>& newParticles, Float initialVelocity_y) {
        if(positionsT0.size() == 0) {
            positionsT0 = newParticles;
        }
        positions.insert(positions.end(), newParticles.begin(), newParticles.end());
        velocities.resize(size(), Vector2(0, -initialVelocity_y));
        affineMat.resize(size(), Matrix2x2(0));
        tmp.resize(size(), Vector2(0));
    }

    void reset() {
        positions.resize(0);
        velocities.resize(0);
        affineMat.resize(0);
        tmp.resize(0);
    }

    template<class Function>
    void loopAll(Function&& func) const {
        for(UnsignedInt p = 0, pend = size(); p < pend; ++p) {
            func(p);
        }
    }

    const Float            particleRadius;
    std::vector<Vector2>   positionsT0;
    std::vector<Vector2>   positions;
    std::vector<Vector2>   velocities;
    std::vector<Matrix2x2> affineMat;
    std::vector<Vector2>   tmp;
};
} }

#endif
