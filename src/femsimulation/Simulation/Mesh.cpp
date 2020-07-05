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

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/MeshData.h>

#include <fstream>

#include "../arcball/ArcBallCamera.h"
#include "Simulation/Mesh.h"

namespace Magnum { namespace Examples {
void TetMesh::loadMesh(const String& meshFile) {
    m_triangles.clear();
    m_tets.clear();

    std::ifstream infile(meshFile);
    if(!infile.is_open()) {
        Fatal{} << "Cannot read file" << meshFile.c_str();
        return;
    }

    char   buffer[256];
    char   ignore[256];
    Vec3f  pos;
    Vec3ui face;
    Vec4ui tet;
    while(!infile.eof()) {
        infile >> buffer;
        if(strcmp(buffer, "#") == 0) {
            /* Skip the line because it is a comment */
            infile.ignore(1000, infile.widen('\n'));
            continue;
        }
        if(strcmp(buffer, "Vertices") == 0) {
            infile >> m_numVerts;
            m_positions_t0.resize(m_numVerts * 3);
            for(u32 i = 0; i < m_numVerts; ++i) {
                infile >> pos[0] >> pos[1] >> pos[2] >> ignore;
                m_positions_t0.block3(i) = pos * 30;
            }
        } else if(strcmp(buffer, "Triangles") == 0) {
            u32 numFaces;
            infile >> numFaces;
            for(u32 i = 0; i < numFaces; ++i) {
                infile >> face[0] >> face[1] >> face[2] >> ignore;
                /* The face id read from file is 1-base, thus minus them by one */
                for(size_t j = 0; j < 3; ++j) {
                    --face[j];
                }
                m_triangles.push_back(face);
            }
        } else if(strcmp(buffer, "Tetrahedra") == 0) {
            u32 numTets;
            infile >> numTets;
            for(u32 i = 0; i < numTets; ++i) {
                infile >> tet[0] >> tet[1] >> tet[2] >> tet[3] >> ignore;
                /* The face id read from file is 1-base, thus minus them by one */
                for(size_t j = 0; j < 4; ++j) {
                    --tet[j];
                }
                m_tets.push_back(tet);
            }
        }
    }

    CORRADE_INTERNAL_ASSERT( m_numVerts > 0);
    CORRADE_INTERNAL_ASSERT(m_triangles.size() > 0);
    CORRADE_INTERNAL_ASSERT(     m_tets.size() > 0);

    /* Find the maximum x value */
    float max_x = -1e10f;
    for(u32 idx = 0; idx < m_numVerts; ++idx) {
        const Vec3f& v = m_positions_t0.block3(idx);
        //        if(max_x < v.x()) { max_x = v.x(); }
        if(max_x < v.y()) { max_x = v.y(); }
    }

    m_fixedVerts.clear();
    /* Fix the vertices that have x ~~ max_x */
    for(u32 idx = 0; idx < m_numVerts; ++idx) {
        const Vec3f& v = m_positions_t0.block3(idx);
        //        if(std::abs(max_x - v.x()) < 1e-4f) {
        if(std::abs(max_x - v.y()) < 1e-1f) {
            m_fixedVerts.push_back(idx);
        }
    }

    /* Reset positions/velocites */
    reset();
    Debug() << "Loaded tet mesh from" << meshFile.c_str();
}

void TetMesh::setupShader() {
    m_vertBuffer = GL::Buffer{};
    m_mesh       = GL::Mesh{};

    Containers::ArrayView<const UnsignedInt> indexData(
        reinterpret_cast<const UnsignedInt*>(m_triangles.data()), m_triangles.size() * 3);
    std::pair<Containers::Array<char>, MeshIndexType> compressed =
        MeshTools::compressIndices(indexData);
    GL::Buffer indices;
    indices.setData(compressed.first);

    m_mesh.setCount(m_triangles.size() * 3)
        .addVertexBuffer(m_vertBuffer, 0, Shaders::Generic3D::Position{})
        .setIndexBuffer(std::move(indices), 0, compressed.second);

    /* https://doc.magnum.graphics/magnum/namespaceMagnum_1_1DebugTools_1_1ColorMap.html */
    const auto     map = DebugTools::ColorMap::viridis();
    const Vector2i size{ Int(map.size()), 1 };
    m_colormap = GL::Texture2D{};
    m_colormap
        .setMinificationFilter(SamplerFilter::Linear)
        .setMagnificationFilter(SamplerFilter::Linear)
        .setWrapping(SamplerWrapping::ClampToEdge)
        .setStorage(1, GL::TextureFormat::RGB8, size)
        .setSubImage(0, {}, ImageView2D{ PixelFormat::RGB8Unorm, size, map });

    m_shader.setColor(Color3{ 0, 1, 1 })
        .setWireframeColor(Color3{ 1, 0, 0 })
        .setWireframeWidth(1.5f);
    //        .setColorMapTransformation(0.0f, 1.0f / m_numVerts);

    //        .bindColorMapTexture(m_colormap);;
}

void TetMesh::draw(ArcBallCamera* camera, const Vector2& viewportSize) {
    Containers::ArrayView<const float> data(reinterpret_cast<const float*>(m_positions.data()),
                                            m_positions.size());
    m_vertBuffer.setData(data, GL::BufferUsage::DynamicDraw);
    m_shader.setTransformationMatrix(camera->viewMatrix())
        .setProjectionMatrix(camera->camera().projectionMatrix())
        .setViewportSize(viewportSize)
        .draw(m_mesh);
}

void TetMesh::reset() {
    m_positions = m_positions_t0;
    m_velocities.resize(3 * m_numVerts);
    m_velocities.setZero();
}
} }
