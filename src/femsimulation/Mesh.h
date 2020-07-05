#ifndef Magnum_Examples_FEMSimulationExample_Mesh_h
#define Magnum_Examples_FEMSimulationExample_Mesh_h
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

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Shaders/MeshVisualizer.h>

#include "MathDefinitions.h"

namespace Magnum { namespace Examples {
class ArcBallCamera;
class Simulator;

class TetMesh {
    friend class Simulator;
public:
    TetMesh(const char* meshFile, Float totalMass = 1.0f) : m_totalMass(totalMass) {
        loadMesh(meshFile);
        setupShader();
    }

    void loadMesh(const char* meshFile);
    void setupShader();
    void draw(ArcBallCamera* camera, const Vector2& viewportSize);
    void reset();

private:
    /* Render helpers */
    GL::Mesh                  m_mesh{ NoCreate };
    GL::Buffer                m_vertBuffer{ NoCreate };
    Shaders::MeshVisualizer3D m_shader{ Shaders::MeshVisualizer3D::Flag::Wireframe };

    /* Public mesh data */
public:
    UnsignedInt m_numVerts; /* n */
    Float       m_totalMass { 1.0f };

    VecXf           m_positions_t0;  /* 1x3n  */
    VecXf           m_positions;     /* 1x3n  */
    VecXf           m_velocities;    /* 1x3n  */
    DiagonalMatrixf m_massMatrix;    /* 3nx3n */
    DiagonalMatrixf m_invMassMatrix; /* 3nx3n */
    DiagonalMatrixf m_massMatrix1D;  /* nxn */

    StdVT<UnsignedInt> m_fixedVerts;
    StdVT<Vec3ui>      m_triangles;
    StdVT<Vec4ui>      m_tets;
};
} }

#endif
