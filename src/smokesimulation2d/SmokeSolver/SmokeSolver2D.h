#ifndef Magnum_Examples_SmokeSimulation2D_SmokeSolver2D_h
#define Magnum_Examples_SmokeSimulation2D_SmokeSolver2D_h
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

#include "Pez.h"

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector2.h>

namespace Magnum { namespace Examples {
typedef struct Surface_ {
    GLuint FboHandle;
    GLuint TextureHandle;
    int    NumComponents;
} Surface;

typedef struct Slab_ {
    Surface Ping;
    Surface Pong;
} Slab;

#define CellSize       (1.25f)
#define ViewportWidth  (PEZ_VIEWPORT_WIDTH)
#define ViewportHeight (PEZ_VIEWPORT_HEIGHT)
#define GridWidth      (ViewportWidth / 2)
#define GridHeight     (ViewportHeight / 2)
#define SplatRadius    ((float)GridWidth / 8.0f)

static const float   AmbientTemperature  = 0.0f;
static const float   ImpulseTemperature  = 5.0f;
static const float   ImpulseDensity      = 2.0f;
static const int     NumJacobiIterations = 40;
static const float   TimeStep               = 0.125f;
static const float   SmokeBuoyancy          = 2.0f;
static const float   SmokeWeight            = 0.05f;
static const float   GradientScale          = 1.125f / CellSize;
static const float   TemperatureDissipation = 0.99f;
static const float   VelocityDissipation    = 0.99f;
static const float   DensityDissipation     = 0.9999f;
static const Vector2 ImpulsePosition        = Vector2 { GridWidth / 2, -(int)SplatRadius / 2 };

static const int PositionSlot = 0;

class SmokeSolver2D {
public:
    GLuint  QuadVao;
    GLuint  VisualizeProgram;
    Slab    Velocity, Density, Pressure, Temperature;
    Surface Divergence, Obstacles, HiresObstacles;

    void PezInitialize();
    void PezUpdate();
    void PezRender(GLuint windowFbo);
private:
    GLuint  CreateProgram(const char* vsKey, const char* gsKey, const char* fsKey);
    Surface CreateSurface(GLsizei width, GLsizei height, int numComponents);
    GLuint  CreateQuad();
    Slab    CreateSlab(GLsizei width, GLsizei height, int numComponents);
    void    CreateObstacles(Surface dest, int width, int height);
    void    InitSlabOps();
    void    SwapSurfaces(Slab* slab);
    void    ClearSurface(Surface s, float value);
    void    Advect(Surface velocity, Surface source, Surface obstacles, Surface dest, float dissipation);
    void    Jacobi(Surface pressure, Surface divergence, Surface obstacles, Surface dest);
    void    SubtractGradient(Surface velocity, Surface pressure, Surface obstacles, Surface dest);
    void    ComputeDivergence(Surface velocity, Surface obstacles, Surface dest);
    void    ApplyImpulse(Surface dest, Vector2 position, float value);
    void    ApplyBuoyancy(Surface velocity, Surface temperature, Surface density, Surface dest);
};
} }
#endif
