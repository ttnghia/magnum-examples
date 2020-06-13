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

#include "Cloth.h"

#include <unordered_map>
#include <unordered_set>

namespace Magnum { namespace Examples {
void Cloth::resetState() {
    positions = positionsT0;
    velocities.assign(velocities.size(), Vector3{ 0 });
}

void Cloth::setCloth(const Vector3& corner, Float w, Float h,
                     UnsignedInt nx, UnsignedInt ny, UnsignedInt bendingSteps) {
    /* Clear fixed vertices */
    fixedVertices.clear();

    Float spacingX = w / Float(nx - 1);
    Float spacingY = h / Float(ny - 1);
    minVertexDistance = Math::min(spacingX, spacingY);

    /* Generate geometry */
    positions.resize(nx * ny);
    texUV.resize(nx * ny);
    faces.resize((nx - 1) * (ny - 1) * 2);
    bool row_flip = false, column_flip = false;
    for(UnsignedInt i = 0; i < nx; ++i) {
        for(UnsignedInt k = 0; k < ny; ++k) {
            /* Generate position and tex coordinate */
            UnsignedInt   index = ny * i + k;
            const Vector3 pos   = corner + Vector3{ spacingX* i, spacingY* k, 0 };
            positions[index] = pos;
            texUV[index]     = (pos - corner).xy() / Vector2{ w, h };

            /* Generate triangles */
            index = (ny - 1) * i + k;

            /* First triangle */
            faces[2 * index ] = { ny* i + k,
                                  ny* i + k + 1,
                                  ny* (i + 1) + ((row_flip ^ column_flip) ? (k + 1) : k) };

            /* Second triangle */
            faces[2 * index + 1] = { ny* (i + 1) + k + 1,
                                     ny* (i + 1) + k,
                                     ny* i + ((row_flip ^ column_flip) ? k : (k + 1)) };

            row_flip = !row_flip;
        }
        column_flip = !column_flip;
        row_flip    = false;
    }

    /* Store positions, and set velocities */
    positionsT0 = positions;
    velocities.assign(positions.size(), Vector3{ 0 });

    /* Generate stretching springs */
    std::unordered_map<UnsignedInt, std::unordered_set<UnsignedInt>> edges;
    stretchingSprings.clear();
    for(const auto& face: faces) {
        UnsignedInt v1 = face.v[2];
        for(UnsignedInt i = 0; i < 3; ++i) {
            UnsignedInt v2 = face.v[i];
            if(v1 < v2
               && (edges.find(v1) == edges.end()
                   || edges[v1].find(v2) == edges[v1].end())) {
                stretchingSprings.push_back({ v1, v2, (positions[v1] - positions[v2]).length() });
                edges[v1].insert(v2);
            }
            v1 = v2;
        }
    }

    /* Generate bending springs */
    bendingSprings.clear();
    for(UnsignedInt i = 0; i < nx; ++i) {
        for(UnsignedInt k = 0; k < ny; ++k) {
            UnsignedInt v1 = ny * i + k;
            UnsignedInt v2;
            Vector3     p1 = positions[v1];
            Vector3     p2;
            if(i + 2 < nx) {
                v2 = ny * (i + 2) + k;
                p2 = positions[v2];
                bendingSprings.push_back({ v1, v2, (p1 - p2).length() });
            }
            if(k + 2 < ny) {
                v2 = ny * i + k + 2;
                p2 = positions[v2];
                bendingSprings.push_back({ v1, v2, (p1 - p2).length() });
            }

            for(int step = 1; step <= bendingSteps; ++step) {
                if(i + step < nx &&
                   k + step < ny) {
                    v1 = ny * i + k;
                    v2 = ny * (i + step) + k + step;
                    p1 = positions[v1];
                    p2 = positions[v2];
                    bendingSprings.push_back({ v1, v2, (p1 - p2).length() });

                    v1 = ny * (i + step) + k;
                    v2 = ny * i + k + step;
                    p1 = positions[v1];
                    p2 = positions[v2];
                    bendingSprings.push_back({ v1, v2, (p1 - p2).length() });
                }
            }
        }
    }
}
} }
