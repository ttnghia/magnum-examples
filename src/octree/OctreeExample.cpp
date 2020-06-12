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
#include <Corrade/Utility/StlMath.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Timeline.h>
#include <Magnum/Trade/MeshData.h>

#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Shaders/Phong.h>

#include "DrawableObjects/ParticleGroup2D.h"
#include "DrawableObjects/WireframeObject2D.h"
#include "FluidSolver/ApicSolver2D.h"

#include "../motionblur/Icosphere.h"

namespace Magnum { namespace Examples {
class OctreeExample : public Platform::Application {
public:
    explicit OctreeExample(const Arguments& arguments);

protected:
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void drawEvent() override;

    /* Scene and drawable group must be constructed before camera and other
       drawble objects */
    Containers::Pointer<Scene3D>                     scene;
    Containers::Pointer<SceneGraph::DrawableGroup3D> drawables;

    /* Camera helpers */
    Containers::Pointer<Object3D>             _objCamera3D;
    Containers::Pointer<SceneGraph::Camera3D> _camera3D;

    Object3D*                           spheres[3];
    Containers::Pointer<GL::Mesh>       mesh;
    Containers::Pointer<Shaders::Phong> shader;

    bool _pausedMotion = false;
};

using namespace Math::Literals;

OctreeExample::OctreeExample(const Arguments& arguments) : Platform::Application{arguments, NoCreate} {
    /* Setup window */
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
        scene.emplace();
        drawables.emplace();

        /* Configure camera */
        _objCamera3D.emplace(scene.get());
        _objCamera3D->translate(Vector3::zAxis(3.0f));

        _camera3D.emplace(*_objCamera3D);
        _camera3D->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 4.0f / 3.0f, 0.001f, 1000.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());
    }

    mesh.emplace();
    *mesh = MeshTools::compile(Primitives::icosphereSolid(3));
    shader.emplace();

    /* Setup point (render as spheres) */ {
        spheres[0] = new Object3D(scene.get());
        (new Icosphere(mesh.get(), shader.get(), 0xff0000_rgbf, spheres[0], drawables.get()))
            ->translate(Vector3::yAxis(0.25f));
        (new Icosphere(mesh.get(), shader.get(), 0xff0000_rgbf, spheres[0], drawables.get()))
            ->translate(Vector3::yAxis(0.25f))
            .rotateZ(120.0_degf);
        (new Icosphere(mesh.get(), shader.get(), 0xff0000_rgbf, spheres[0], drawables.get()))
            ->translate(Vector3::yAxis(0.25f))
            .rotateZ(240.0_degf);

        spheres[1] = new Object3D(scene.get());
        (new Icosphere(mesh.get(), shader.get(), 0x00ff00_rgbf, spheres[1], drawables.get()))
            ->translate(Vector3::yAxis(0.50f));
        (new Icosphere(mesh.get(), shader.get(), 0x00ff00_rgbf, spheres[1], drawables.get()))
            ->translate(Vector3::yAxis(0.50f))
            .rotateZ(120.0_degf);
        (new Icosphere(mesh.get(), shader.get(), 0x00ff00_rgbf, spheres[1], drawables.get()))
            ->translate(Vector3::yAxis(0.50f))
            .rotateZ(240.0_degf);

        spheres[2] = new Object3D(scene.get());
        (new Icosphere(mesh.get(), shader.get(), 0x0000ff_rgbf, spheres[2], drawables.get()))
            ->translate(Vector3::yAxis(0.75f));
        (new Icosphere(mesh.get(), shader.get(), 0x0000ff_rgbf, spheres[2], drawables.get()))
            ->translate(Vector3::yAxis(0.75f))
            .rotateZ(120.0_degf);
        (new Icosphere(mesh.get(), shader.get(), 0x0000ff_rgbf, spheres[2], drawables.get()))
            ->translate(Vector3::yAxis(0.75f))
            .rotateZ(240.0_degf);
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    /* Start the timer, loop at 60 Hz max */
    setSwapInterval(1);
    setMinimalLoopPeriod(16);
}

void OctreeExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    if(!_pausedMotion) {
        //
    }
    _camera3D->draw(*drawables);

    swapBuffers();
    /* Run next frame immediately */
    redraw();
}

void OctreeExample::viewportEvent(ViewportEvent& event) {
    /* Resize the main framebuffer */
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });

    /* Recompute the camera's projection matrix */
    _camera3D->setViewport(event.framebufferSize());
}

void OctreeExample::keyPressEvent(KeyEvent& event) {
    switch(event.key()) {
        case KeyEvent::Key::R:
            event.setAccepted(true);
            break;
        case KeyEvent::Key::Space:
            _pausedMotion ^= true;
            event.setAccepted(true);
            break;
        default:;
    }
}
} }

MAGNUM_APPLICATION_MAIN(Magnum::Examples::OctreeExample)
