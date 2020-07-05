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
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Debug.h>
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
void TetMesh::loadMesh(const char* meshFile) {
    arrayResize(_triangles, 0);
    arrayResize(_tets,      0);

    std::ifstream infile(meshFile);
    if(!infile.is_open()) {
        Fatal{} << "Cannot read file" << meshFile;
        return;
    }

    char      buffer[256];
    char      ignore[256];
    EgVec3f   pos;
    Vector3ui face;
    Vector4ui tet;
    while(!infile.eof()) {
        infile >> buffer;
        if(strcmp(buffer, "#") == 0) {
            /* Skip the line because it is a comment */
            infile.ignore(1000, infile.widen('\n'));
            continue;
        }
        if(strcmp(buffer, "Vertices") == 0) {
            infile >> _numVerts;
            _positions_t0.resize(_numVerts * 3);
            for(UnsignedInt i = 0; i < _numVerts; ++i) {
                infile >> pos[0] >> pos[1] >> pos[2] >> ignore;
                _positions_t0.block3(i) = pos * 1;
            }
        } else if(strcmp(buffer, "Triangles") == 0) {
            UnsignedInt numFaces;
            infile >> numFaces;
            for(UnsignedInt i = 0; i < numFaces; ++i) {
                infile >> face[0] >> face[1] >> face[2] >> ignore;
                /* The face id read from file is 1-base, thus minus them by one */
                for(size_t j = 0; j < 3; ++j) {
                    --face[j];
                }
                arrayAppend(_triangles, face);
            }
        } else if(strcmp(buffer, "Tetrahedra") == 0) {
            UnsignedInt numTets;
            infile >> numTets;
            for(UnsignedInt i = 0; i < numTets; ++i) {
                infile >> tet[0] >> tet[1] >> tet[2] >> tet[3] >> ignore;
                /* The face id read from file is 1-base, thus minus them by one */
                for(size_t j = 0; j < 4; ++j) {
                    --tet[j];
                }
                arrayAppend(_tets, tet);
            }
        }
    }

    CORRADE_INTERNAL_ASSERT(_numVerts > 0);
    CORRADE_INTERNAL_ASSERT(_triangles.size() > 0);
    CORRADE_INTERNAL_ASSERT(_tets.size() > 0);

    /* Reset positions/velocites */
    reset();
    Debug() << "Loaded tet mesh from" << meshFile;
}

void TetMesh::setupShader() {
    m_vertBuffer = GL::Buffer{};
    m_mesh       = GL::Mesh{};

    Containers::ArrayView<const UnsignedInt> indexData(
        reinterpret_cast<const UnsignedInt*>(_triangles.data()), _triangles.size() * 3);
    std::pair<Containers::Array<char>, MeshIndexType> compressed =
        MeshTools::compressIndices(indexData);
    GL::Buffer indices;
    indices.setData(compressed.first);

    m_mesh.setCount(_triangles.size() * 3)
        .addVertexBuffer(m_vertBuffer, 0, Shaders::Generic3D::Position{})
        .setIndexBuffer(std::move(indices), 0, compressed.second);

    /* Please select a good color */
    //    m_shader.setColor(Color3{ 0, 1, 1 })
    //    m_shader.setColor(Color3{ Vector3{ 3, 252, 248 } / 255.0 })
    //    m_shader.setColor(Color3{ Vector3{ 3, 252, 198 } / 255.0 })
    //    m_shader.setColor(Color3{ 0.275f, 0.08f, 0.4f })
    //    m_shader.setColor(Color3{ Vector3{ 252, 223, 3 } / 255.0 })
    //    m_shader.setColor(Color3{ Vector3{ 252, 190, 3 } / 255.0 })
    //    m_shader.setColor(Color3{ Vector3{ 252, 115, 3 } / 255.0 })
    m_shader.setColor(Color3{ Vector3{ 252, 186, 3 } / 255.0 })
        .setWireframeColor(Color3{ 0 })
        .setWireframeWidth(0.5f);
}

void TetMesh::draw(ArcBallCamera* camera, const Vector2& viewportSize) {
    Containers::ArrayView<const Float> data(reinterpret_cast<const Float*>(_positions.data()),
                                            _positions.size());
    m_vertBuffer.setData(data, GL::BufferUsage::DynamicDraw);
    m_shader.setTransformationMatrix(camera->viewMatrix())
        .setProjectionMatrix(camera->camera().projectionMatrix())
        .setViewportSize(viewportSize)
        .draw(m_mesh);
}

void TetMesh::reset() {
    _positions = _positions_t0;
    _velocities.resize(3 * _numVerts);
    _velocities.setZero();
}
} }
