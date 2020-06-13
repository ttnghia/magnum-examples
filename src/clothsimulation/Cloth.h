#ifndef Magnum_Examples_ClothSimulation_Cloth_h
#define Magnum_Examples_ClothSimulation_Cloth_h
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

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Vector3.h>

#include <vector>
#include <unordered_set>

namespace Magnum { namespace Examples {
struct Spring {
    enum class SpringType {
        Constraint,
        Stretching,
        Bending
    };
    SpringType  type;
    UnsignedInt targetVertex;
    Float       restLength;
};

struct Face {
    UnsignedInt v[3];
};

struct Cloth {
    explicit Cloth() = default;
    explicit Cloth(const Vector3& corner, const Vector2& size, const Vector2ui resolution,
                   UnsignedInt bendingSteps = 1) {
        CORRADE_INTERNAL_ASSERT(size.x() > 0 && size.y() > 0 && resolution.x() > 1 && resolution.y() > 1);
        setCloth(corner, size, resolution, bendingSteps);
    }

    void resetState();
    void setCloth(const Vector3& corner, const Vector2& size, const Vector2ui resolution,
                  UnsignedInt bendingSteps = 1);
    void setFixedVertex(UnsignedInt vidx);

    std::size_t getNumVertices() const { return positions.size(); }
    bool isFixedVertex(UnsignedInt vidx) const { return fixedVertices.find(vidx) != fixedVertices.end(); }

    template<class Function>
    void loopVertices(Function&& func) const {
        for(UnsignedInt vidx = 0, vend = getNumVertices(); vidx < vend; ++vidx) {
            func(vidx);
        }
    }

    template<class Function>
    void loopSprings(Function&& func) const {
        for(UnsignedInt sidx = 0, send = getNumSprings(); sidx < send; ++sidx) {
            func(sidx);
        }
    }

    std::unordered_set<UnsignedInt> fixedVertices;

    /* Per vertex data */
    std::vector<Vector3>             positionsT0;
    std::vector<Vector3>             positions;
    std::vector<Vector3>             velocities;
    std::vector<std::vector<Spring>> vertexSprings; /* spring list per vertex */

    /* Spring data */
    Float shortestSpring;

    /* Visualization data */
    std::vector<Vector2> texUV;
    std::vector<Face>    faces;
};
} }

#endif
