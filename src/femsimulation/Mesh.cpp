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
#include "Mesh.h"

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
                m_positions_t0.block3(i) = pos * 1;
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
        if(max_x < v.x()) { max_x = v.x(); }
        //        if(max_x < v.y()) { max_x = v.y(); }
    }

    m_fixedVerts.clear();
    /* Fix the vertices that have x ~~ max_x */
    for(u32 idx = 0; idx < m_numVerts; ++idx) {
        const Vec3f& v = m_positions_t0.block3(idx);
        if(std::abs(max_x - v.x()) < 1e-4f) {
            //        if(std::abs(max_x - v.y()) < 1e-1f) {
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

    m_shader.setColor(Color3{ 0.275, 0.08, 0.4 })
        .setWireframeColor(Color3{ 1, 1, 1 })
        .setWireframeWidth(0.5f);
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
