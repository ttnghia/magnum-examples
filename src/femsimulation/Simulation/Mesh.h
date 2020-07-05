/**
 * Copyright 2020 Nghia Truong <nghiatruong.vn@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "Camera/ArcBallCamera.h"
#include "Setup.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Shaders/MeshVisualizer.h>

/****************************************************************************************************/
class Simulator;
class TetMesh {
    friend class Simulator;
public:
    TetMesh(const String& meshFile, float totalMass = 1.0f) : m_totalMass(totalMass) {
        loadMesh(meshFile);
        setupShader();
    }

    void loadMesh(const String& meshFile);
    void setupShader();
    void draw(ArcBallCamera* camera, const Vector2& viewportSize);
    void reset();

private:
    /* Render helpers */
    GL::Mesh                  m_mesh{ NoCreate };
    GL::Buffer                m_vertBuffer{ NoCreate };
    GL::Texture2D             m_colormap{ NoCreate };
    Shaders::MeshVisualizer3D m_shader{
        Shaders::MeshVisualizer3D::Flag::Wireframe |
        Shaders::MeshVisualizer3D::Flag::VertexId };

    /* Public mesh data */
public:
    u32   m_numVerts;   /* n */
    float m_totalMass { 1.0f };

    VecXf           m_positions_t0;  /* 1x3n  */
    VecXf           m_positions;     /* 1x3n  */
    VecXf           m_velocities;    /* 1x3n  */
    DiagonalMatrixf m_massMatrix;    /* 3nx3n */
    DiagonalMatrixf m_invMassMatrix; /* 3nx3n */
    DiagonalMatrixf m_massMatrix1D;  /* nxn */

    StdVT<u32>    m_fixedVerts;
    StdVT<Vec3ui> m_triangles;
    StdVT<Vec4ui> m_tets;
};
