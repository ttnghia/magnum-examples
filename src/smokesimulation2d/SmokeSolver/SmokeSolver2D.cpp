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

#include "SmokeSolver/SmokeSolver2D.h"

namespace Magnum { namespace Examples {
const char* PezInitialize(int width, int height) {
    int w = GridWidth;
    int h = GridHeight;
    Velocity    = CreateSlab(w, h, 2);
    Density     = CreateSlab(w, h, 1);
    Pressure    = CreateSlab(w, h, 1);
    Temperature = CreateSlab(w, h, 1);
    Divergence  = CreateSurface(w, h, 3);
    InitSlabOps();
    VisualizeProgram = CreateProgram("Fluid.Vertex", 0, "Fluid.Visualize");

    Obstacles = CreateSurface(w, h, 3);
    CreateObstacles(Obstacles, w, h);

    w = ViewportWidth * 2;
    h = ViewportHeight * 2;
    HiresObstacles = CreateSurface(w, h, 1);
    CreateObstacles(HiresObstacles, w, h);

    QuadVao = CreateQuad();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    ClearSurface(Temperature.Ping, AmbientTemperature);
    return "Fluid Demo";
}

void PezUpdate(unsigned int elapsedMicroseconds) {
    glViewport(0, 0, GridWidth, GridHeight);

    Advect(Velocity.Ping, Velocity.Ping, Obstacles, Velocity.Pong, VelocityDissipation);
    SwapSurfaces(&Velocity);

    Advect(Velocity.Ping, Temperature.Ping, Obstacles, Temperature.Pong, TemperatureDissipation);
    SwapSurfaces(&Temperature);

    Advect(Velocity.Ping, Density.Ping, Obstacles, Density.Pong, DensityDissipation);
    SwapSurfaces(&Density);

    ApplyBuoyancy(Velocity.Ping, Temperature.Ping, Density.Ping, Velocity.Pong);
    SwapSurfaces(&Velocity);

    ApplyImpulse(Temperature.Ping, ImpulsePosition, ImpulseTemperature);
    ApplyImpulse(Density.Ping,     ImpulsePosition, ImpulseDensity);

    ComputeDivergence(Velocity.Ping, Obstacles, Divergence);
    ClearSurface(Pressure.Ping, 0);

    for(int i = 0; i < NumJacobiIterations; ++i) {
        Jacobi(Pressure.Ping, Divergence, Obstacles, Pressure.Pong);
        SwapSurfaces(&Pressure);
    }

    SubtractGradient(Velocity.Ping, Pressure.Ping, Obstacles, Velocity.Pong);
    SwapSurfaces(&Velocity);
}

void PezRender(GLuint windowFbo) {
    // Bind visualization shader and set up blend state:
    glUseProgram(VisualizeProgram);
    GLint fillColor = glGetUniformLocation(VisualizeProgram, "FillColor");
    GLint scale     = glGetUniformLocation(VisualizeProgram, "Scale");
    glEnable(GL_BLEND);

    // Set render target to the backbuffer:
    glViewport(0, 0, ViewportWidth, ViewportHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, windowFbo);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw ink:
    glBindTexture(GL_TEXTURE_2D, Density.Ping.TextureHandle);
    glUniform3f(fillColor, 1, 1, 1);
    glUniform2f(scale, 1.0f / ViewportWidth, 1.0f / ViewportHeight);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // Draw obstacles:
    glBindTexture(GL_TEXTURE_2D, HiresObstacles.TextureHandle);
    glUniform3f(fillColor, 0.125f, 0.4f, 0.75f);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // Disable blending:
    glDisable(GL_BLEND);
}

void PezHandleMouse(int x, int y, int action) {}
} }
