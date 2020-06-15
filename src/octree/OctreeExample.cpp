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

#include <Corrade/Containers/Pointer.h>
#include <Corrade/Utility/Arguments.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Trade/MeshData.h>

#include "LooseOctree.h"
#include "../arcball/ArcBallCamera.h"
#include "../motionblur/Icosphere.h"
#include "../fluidsimulation3d/DrawableObjects/WireframeObjects.h"

namespace Magnum { namespace Examples {
class OctreeExample : public Platform::Application {
public:
    explicit OctreeExample(const Arguments& arguments);

protected:
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    void updatePoints();
    void collisionDetectionAndHandlingBruteForce();
    void collisionDetectionAndHandlingUsingOctree();
    void checkCollisionWithSubTree(const OctreeNode* const pNode, std::size_t i,
                                   const Vector3& ppos, const Vector3& pvel);
    void updateTreeNodeBoundingBoxes();

    /* Scene and drawable group must be constructed before camera and other
       drawble objects */
    Containers::Pointer<Scene3D>                     _scene;
    Containers::Pointer<SceneGraph::DrawableGroup3D> _drawables;
    Containers::Pointer<ArcBallCamera>               _arcballCamera;

    /* Render points as spheres with size */
    Containers::Pointer<GL::Mesh>       _mesh;
    Containers::Pointer<Shaders::Phong> _shader;

    std::vector<Vector3>   _spheresPos;
    std::vector<Vector3>   _spheresVel;
    std::vector<Object3D*> _spheres;
    Float                  _sphereRadius;
    bool                   _bAnimation = true;

    /* Octree and boundary boxes */
    Containers::Pointer<LooseOctree> _octree;
    WireframeBox*                    _rootNodeBoundingBox;
    std::vector<WireframeBox*>       _treeNodeBoundingBoxes;
    bool _bDrawBoundingBoxes = true;
};

using namespace Math::Literals;

OctreeExample::OctreeExample(const Arguments& arguments) : Platform::Application{arguments, NoCreate} {
    Utility::Arguments args;
    args.addOption("num-spheres", "10")
        .setHelp("num-spheres", "number of spheres to simulate", "SPHERES")
        .addOption("sphere-radius", "0.05")
        .setHelp("sphere-radius", "radius of the spheres", "RADIUS")
        .addOption("sphere-velocity", "1.0")
        .setHelp("sphere-velocity", "velocity of the spheres", "VELOCITY")
        .parse(arguments.argc, arguments.argv);

    /* Setup window and parameters */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Magnum Octree Example")
            .setSize(conf.size(), dpiScaling)
            .setWindowFlags(Configuration::WindowFlag::Resizable);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf)) {
            create(conf, glConf.setSampleCount(0));
        }
    }

    /* Setup scene objects and camera */
    {
        /* Setup scene objects */
        _scene.emplace();
        _drawables.emplace();

        /* Configure camera */
        const Vector3 eye{ Vector3::zAxis(5.0f) };
        const Vector3 viewCenter{ };
        const Vector3 up{ Vector3::yAxis() };
        const Deg     fov = 45.0_degf;
        _arcballCamera.emplace(*_scene.get(), eye, viewCenter, up, fov, windowSize(), framebufferSize());
        _arcballCamera->setLagging(0.85f);
    }

    /* Setup point (render as spheres) */
    {
        _shader.emplace();
        _mesh.emplace();
        *_mesh = MeshTools::compile(Primitives::icosphereSolid(3));

        const UnsignedInt numSpheres = args.value<UnsignedInt>("num-spheres");
        _spheresPos.resize(numSpheres);
        _spheresVel.resize(numSpheres);
        _spheres.resize(numSpheres);

        for(std::size_t i = 0; i < numSpheres; ++i) {
            const Vector3 tmpPos{ std::rand() / Float(RAND_MAX),
                                  std::rand() / Float(RAND_MAX),
                                  std::rand() / Float(RAND_MAX) };
            const Vector3 tmpVel{ std::rand() / Float(RAND_MAX),
                                  std::rand() / Float(RAND_MAX),
                                  std::rand() / Float(RAND_MAX) };
            const Vector3 pos = tmpPos * 2 - Vector3{ 1 };
            const Vector3 vel = (tmpVel * 2 - Vector3{ 1 }).normalized();
            _spheres[i] = new Object3D(_scene.get());
            _spheres[i]->setTransformation(Matrix4::translation(pos));
            _spheresPos[i] = pos;
            _spheresVel[i] = vel * args.value<Float>("sphere-velocity");

            /* Icosphere is already scaled by 0.1, thus we must multiply by 10 to get a unit sphere */
            _sphereRadius = args.value<Float>("sphere-radius");
            (new Icosphere(_mesh.get(), _shader.get(), Color3{ tmpPos }, _spheres[i], _drawables.get()))
                ->scale(Vector3{ 10.0f * _sphereRadius });
        }
    }

    /* Setup octree */
    {
        _octree.emplace(Vector3{ 0 }, 1.0f, 0.1f);
        _octree->setPoints(_spheresPos);
        _octree->build();

        /* Add a box for drawing the root node with a different color */
        _rootNodeBoundingBox = new WireframeBox(_scene.get(), _drawables.get());
        _rootNodeBoundingBox->setTransformation(
            Matrix4::translation(_octree->getRootNode()->getCenter()) *
            Matrix4::scaling(Vector3{ _octree->getRootNode()->getHalfWidth()*1.001f })
            ).setColor(Color3(0, 1, 1));

        /* Draw the remaining nodes */
        updateTreeNodeBoundingBoxes();
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    /* Start the timer, loop at 60 Hz max */
    setSwapInterval(1);
    setMinimalLoopPeriod(16);
}

void OctreeExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    if(_bAnimation) {
        collisionDetectionAndHandlingBruteForce();
        collisionDetectionAndHandlingUsingOctree();
        updatePoints();
        _octree->updatePoints(_spheresPos);
        _octree->update();
        updateTreeNodeBoundingBoxes();
    }

    _arcballCamera->update();
    _arcballCamera->draw(*_drawables);

    swapBuffers();
    /* Run next frame immediately */
    redraw();
}

void OctreeExample::collisionDetectionAndHandlingBruteForce() {
    for(std::size_t i = 0; i < _spheresPos.size(); ++i) {
        const Vector3 ppos = _spheresPos[i];
        const Vector3 pvel = _spheresVel[i];
        for(std::size_t j = i + 1; j < _spheresPos.size(); ++j) {
            const Vector3 qpos      = _spheresPos[j];
            const Vector3 qvel      = _spheresVel[j];
            const Vector3 velpq     = pvel - qvel;
            const Vector3 pospq     = ppos - qpos;
            const Float   dpq       = pospq.length();
            const Vector3 pospqNorm = pospq / dpq;
            const Float   vp        = Math::dot(velpq, pospqNorm);
            if(vp < 0 && dpq < 2 * _sphereRadius) {
                const Vector3 vNormal = vp * pospqNorm;
                _spheresVel[i] = (_spheresVel[i] - vNormal).normalized();
                _spheresVel[j] = (_spheresVel[j] + vNormal).normalized();
            }
        }
    }
}

void OctreeExample::collisionDetectionAndHandlingUsingOctree() {
    const OctreeNode* const rootNode = _octree->getRootNode();
    for(std::size_t i = 0; i < _spheresPos.size(); ++i) {
        const Vector3 ppos = _spheresPos[i];
        const Vector3 pvel = _spheresVel[i];
        checkCollisionWithSubTree(rootNode, i, ppos, pvel);
    }
}

void OctreeExample::checkCollisionWithSubTree(const OctreeNode* const pNode, std::size_t i,
                                              const Vector3& ppos, const Vector3& pvel) {
    if(!pNode->looselyContains(ppos)) {
        return;
    }

    if(!pNode->isLeaf()) {
        for(std::size_t childIdx = 0; childIdx < 8; childIdx++) {
            const OctreeNode* const pChildNode = pNode->getChildNode(childIdx);
            checkCollisionWithSubTree(pChildNode, i, ppos, pvel);
        }
    }

    const OctreePoint* pIter = pNode->getPointList();
    while(pIter) {
        const std::size_t j = pIter->idx;
        if(j != i) {
            const Vector3 qpos      = _spheresPos[j];
            const Vector3 qvel      = _spheresVel[j];
            const Vector3 velpq     = pvel - qvel;
            const Vector3 pospq     = ppos - qpos;
            const Float   dpq       = pospq.length();
            const Vector3 pospqNorm = pospq / dpq;
            const Float   vp        = Math::dot(velpq, pospqNorm);
            if(vp < 0 && dpq < 2 * _sphereRadius) {
                const Vector3 vNormal = vp * pospqNorm;
                _spheresVel[i] = (_spheresVel[i] - vNormal).normalized();
                _spheresVel[j] = (_spheresVel[j] + vNormal).normalized();
            }
        }
        pIter = pIter->pNext;
    }
}

void OctreeExample::updatePoints() {
    static constexpr Float dt{ 1.0f / 300.0f };

    for(std::size_t i = 0; i < _spheresPos.size(); ++i) {
        Vector3 pos = _spheresPos[i] + _spheresVel[i] * dt;
        for(std::size_t j = 0; j < 3; ++j) {
            if(pos[j] < -1.0f || pos[j] > 1.0f) {
                _spheresVel[i][j] = -_spheresVel[i][j];
            }
            pos[j] = Math::clamp(pos[j], -1.0f, 1.0f);
        }
        _spheresPos[i] = pos;
        _spheres[i]->setTransformation(Matrix4::translation(pos));
    }
}

void OctreeExample::updateTreeNodeBoundingBoxes() {
    if(!_bDrawBoundingBoxes) {
        return;
    }
    const auto& activeTreeNodeBlocks = _octree->getActiveTreeNodeBlocks();
    std::size_t boxIdx{ 0 };

    for(OctreeNodeBlock* const pNodeBlock : activeTreeNodeBlocks) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            OctreeNode* const pNode = &pNodeBlock->_nodes[childIdx];
            if(!pNode->isLeaf() || pNode->getPointCount() > 0) { /* non-empty node */
                if(boxIdx == _treeNodeBoundingBoxes.size()) {
                    auto drawableBox = new WireframeBox(_scene.get(), _drawables.get());
                    drawableBox->setColor(Color3(0.1f, 0.5f, 0.6f));
                    _treeNodeBoundingBoxes.emplace_back(drawableBox);
                }
                _treeNodeBoundingBoxes[boxIdx++]->setTransformation(
                    Matrix4::translation(pNode->getCenter()) *
                    Matrix4::scaling(Vector3{ pNode->getHalfWidth() })
                    );
            }
        }
    }

    /* For the remaining boxes, hide them away */
    while(boxIdx < _treeNodeBoundingBoxes.size()) {
        _treeNodeBoundingBoxes[boxIdx++]->setTransformation(
            Matrix4::translation(Vector3{ 100 }) *
            Matrix4::scaling(Vector3{ 0 })
            );
    }
}

void OctreeExample::viewportEvent(ViewportEvent& event) {
    /* Resize the main framebuffer */
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });
    _arcballCamera->reshape(event.windowSize(), event.framebufferSize());
}

void OctreeExample::keyPressEvent(KeyEvent& event) {
    switch(event.key()) {
        case KeyEvent::Key::B:
            _bDrawBoundingBoxes ^= true;
            if(!_bDrawBoundingBoxes) {
                for(auto& drawableBox: _treeNodeBoundingBoxes) {
                    drawableBox->setTransformation(
                        Matrix4::translation(Vector3{ 100 }) *
                        Matrix4::scaling(Vector3{ 0 })
                        );
                }
            }
            event.setAccepted(true);
            break;
        case KeyEvent::Key::R:
            _arcballCamera->reset();
            event.setAccepted(true);
            break;
        case KeyEvent::Key::Space:
            _bAnimation ^= true;
            event.setAccepted(true);
            break;
        default:;
    }
}

void OctreeExample::mousePressEvent(MouseEvent& event) {
    /* Enable mouse capture so the mouse can drag outside of the window */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_TRUE);

    _arcballCamera->initTransformation(event.position());

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void OctreeExample::mouseReleaseEvent(MouseEvent&) {
    /* Disable mouse capture again */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_FALSE);
}

void OctreeExample::mouseMoveEvent(MouseMoveEvent& event) {
    if(!event.buttons()) { return; }

    if(event.modifiers() & MouseMoveEvent::Modifier::Shift) {
        _arcballCamera->translate(event.position());
    } else { _arcballCamera->rotate(event.position()); }

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void OctreeExample::mouseScrollEvent(MouseScrollEvent& event) {
    const Float delta = event.offset().y();
    if(Math::abs(delta) < 1.0e-2f) { return; }

    _arcballCamera->zoom(delta);

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}
} }

MAGNUM_APPLICATION_MAIN(Magnum::Examples::OctreeExample)
