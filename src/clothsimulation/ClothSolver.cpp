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

        /* Advect particles */
        moveVertices(substep);
    }
}

Float MSSSolver::timestepCFL() const {
    Float maxVel = 0;
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            maxVel = Math::max(maxVel, Math::abs(_cloth.velocities[vidx].length()));
                        });
    return maxVel > 0 ? _cflFactor / maxVel : 1.0f;
}

void MSSSolver::moveVertices(Float dt) {
    _cloth.loopVertices([&](UnsignedInt vidx) {
                            if(!_cloth.isFixedVertex(vidx)) {
                                _cloth.positions[vidx] += _cloth.velocities[vidx] * dt;
                            }
                        });
}
} }
