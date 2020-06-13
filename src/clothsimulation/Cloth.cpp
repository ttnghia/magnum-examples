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

#include <algorithm>

namespace Magnum { namespace Examples {
void Cloth::resetState() {
    positions = positionsT0;
    velocities.assign(velocities.size(), Vector3{ 0 });
}

void Cloth::setCloth(const Vector3& corner,
                     const Vector2& size, const Vector2ui& resolution,
                     UnsignedInt bendingSteps) {
    /* Clear data */
    fixedVertices.clear();

    Float spacingX = size.x() / Float(resolution.x() - 1);
    Float spacingY = size.y() / Float(resolution.y() - 1);
    shortestSpring = Math::min(spacingX, spacingY);

    /* Generate geometry */
    positions.resize(resolution.x() * resolution.y());
    texUV.resize(resolution.x() * resolution.y());
    faces.resize((resolution.x() - 1) * (resolution.y() - 1) * 2);
    /* Generate position and tex coordinate */
    for(UnsignedInt i = 0; i < resolution.x(); ++i) {
        for(UnsignedInt k = 0; k < resolution.y(); ++k) {
            const UnsignedInt index = resolution.y() * i + k;
            CORRADE_INTERNAL_ASSERT(index < getNumVertices());

            const Vector3 pos = corner + Vector3{ spacingX* i, spacingY* k, 0 };
            positions[index] = pos;
            texUV[index]     = (pos - corner).xy() / size;
        }
    }

    bool row_flip = false, column_flip = false;
    for(UnsignedInt i = 0; i < resolution.x() - 1; ++i) {
        for(UnsignedInt k = 0; k < resolution.y() - 1; ++k) {
            const UnsignedInt index = (resolution.y() - 1) * i + k;
            CORRADE_INTERNAL_ASSERT(2 * index + 1 < faces.size());

            /* First triangle */
            faces[2 * index ] = { resolution.y() * i + k,
                                  resolution.y() * i + k + 1,
                                  resolution.y() * (i + 1) + ((row_flip ^ column_flip) ? (k + 1) : k) };

            /* Second triangle */
            faces[2 * index + 1] = { resolution.y() * (i + 1) + k + 1,
                                     resolution.y() * (i + 1) + k,
                                     resolution.y() * i + ((row_flip ^ column_flip) ? k : (k + 1)) };

            row_flip = !row_flip;
        }
        column_flip = !column_flip;
        row_flip    = false;
    }

    /* Store positions, and set velocities */
    positionsT0 = positions;
    velocities.assign(positions.size(), Vector3{ 0 });

    /* Generate springs */
    vertexSprings.resize(positions.size());
    for(UnsignedInt i = 0; i < resolution.x(); ++i) {
        for(UnsignedInt k = 0; k < resolution.y(); ++k) {
            UnsignedInt v1 = resolution.y() * i + k;
            CORRADE_INTERNAL_ASSERT(v1 < getNumVertices());

            UnsignedInt v2;
            Vector3     p1 = positions[v1];
            Vector3     p2;

            /* Horizontal/vertical stretching springs */
            if(i + 1 < resolution.x()) {
                v2 = resolution.y() * (i + 1) + k;
                CORRADE_INTERNAL_ASSERT(v2 < getNumVertices());
                p2 = positions[v2];
                const auto l0 = (p1 - p2).length();
                vertexSprings[v1].push_back({ Spring::SpringType::Stretching, v2, l0 });
                vertexSprings[v2].push_back({ Spring::SpringType::Stretching, v1, l0 });
            }
            if(k + 1 < resolution.y()) {
                v2 = resolution.y() * i + k + 1;
                CORRADE_INTERNAL_ASSERT(v2 < getNumVertices());
                p2 = positions[v2];
                const auto l0 = (p1 - p2).length();
                vertexSprings[v1].push_back({ Spring::SpringType::Stretching, v2, l0 });
                vertexSprings[v2].push_back({ Spring::SpringType::Stretching, v1, l0 });
            }

            /* Horizontal/vertical bending springs */
            if(i + 2 < resolution.x()) {
                v2 = resolution.y() * (i + 2) + k;
                CORRADE_INTERNAL_ASSERT(v2 < getNumVertices());
                p2 = positions[v2];
                const auto l0 = (p1 - p2).length();
                vertexSprings[v1].push_back({ Spring::SpringType::Bending, v2, l0 });
                vertexSprings[v2].push_back({ Spring::SpringType::Bending, v1, l0 });
            }
            if(k + 2 < resolution.y()) {
                v2 = resolution.y() * i + k + 2;
                CORRADE_INTERNAL_ASSERT(v2 < getNumVertices());
                p2 = positions[v2];
                const auto l0 = (p1 - p2).length();
                vertexSprings[v1].push_back({ Spring::SpringType::Bending, v2, l0 });
                vertexSprings[v2].push_back({ Spring::SpringType::Bending, v1, l0 });
            }

            for(UnsignedInt step = 1; step <= bendingSteps; ++step) {
                if(i + step < resolution.x() &&
                   k + step < resolution.y()) {
                    v1 = resolution.y() * i + k;
                    v2 = resolution.y() * (i + step) + k + step;
                    CORRADE_INTERNAL_ASSERT(v1 < getNumVertices() && v2 < getNumVertices());
                    p1 = positions[v1];
                    p2 = positions[v2];
                    auto l0 = (p1 - p2).length();
                    vertexSprings[v1].push_back({ Spring::SpringType::Bending, v2, l0 });
                    vertexSprings[v2].push_back({ Spring::SpringType::Bending, v1, l0 });

                    v1 = resolution.y() * (i + step) + k;
                    v2 = resolution.y() * i + k + step;
                    CORRADE_INTERNAL_ASSERT(v1 < getNumVertices() && v2 < getNumVertices());
                    p1 = positions[v1];
                    p2 = positions[v2];
                    l0 = (p1 - p2).length();
                    vertexSprings[v1].push_back({ Spring::SpringType::Bending, v2, l0 });
                    vertexSprings[v2].push_back({ Spring::SpringType::Bending, v1, l0 });
                }
            }
        }
    }

    for(auto& springList: vertexSprings) {
        std::sort(springList.begin(), springList.end(),
                  [](const Spring& s1, const Spring& s2) {
                      return s1.targetVertex < s2.targetVertex;
                  });
    }
}

void Cloth::setFixedVertex(UnsignedInt vidx) {
    fixedVertices.insert(vidx);

    auto& springList = vertexSprings[vidx];
    springList.push_back({ Spring::SpringType::Constraint, vidx, 0 });
    std::sort(springList.begin(), springList.end(),
              [](const Spring& s1, const Spring& s2) {
                  return s1.targetVertex < s2.targetVertex;
              });
}
} }
