#ifndef Magnum_Examples_SmokeSimulation2D_SmokeShaders2D_h
#define Magnum_Examples_SmokeSimulation2D_SmokeShaders2D_h
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

#include <Magnum/GL/AbstractShaderProgram.h>

namespace Magnum { namespace Examples {
class FillTextureShader : public GL::AbstractShaderProgram {
public:
    FillTextureShader();
    FillTextureShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class AdvectShader : public GL::AbstractShaderProgram {
public:
    AdvectShader();
    AdvectShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class JacobiShader : public GL::AbstractShaderProgram {
public:
    JacobiShader();
    JacobiShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class SubtractGradientShader : public GL::AbstractShaderProgram {
public:
    SubtractGradientShader();
    SubtractGradientShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class ComputeDivergenceShader : public GL::AbstractShaderProgram {
public:
    ComputeDivergenceShader();
    ComputeDivergenceShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class ApplyImpulseShader : public GL::AbstractShaderProgram {
public:
    ApplyImpulseShader();
    ApplyImpulseShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class ApplyBuoyancyShader : public GL::AbstractShaderProgram {
public:
    ApplyBuoyancyShader();
    ApplyBuoyancyShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};

class SmokeShader : public GL::AbstractShaderProgram {
public:
    SmokeShader();
    SmokeShader& setNumParticles(Int numParticles);

private:
    Int _uNumParticles;
};
} }

#endif
