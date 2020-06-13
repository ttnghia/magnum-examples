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
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Trade/MeshData.h>

#include "../arcball/ArcBallCamera.h"
#include "../motionblur/Icosphere.h"

namespace Magnum { namespace Examples {
class FEMSimulationExample : public Platform::Application {
public:
    explicit FEMSimulationExample(const Arguments& arguments);

protected:
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    /* Scene and drawable group must be constructed before camera and other
       drawble objects */
    Containers::Pointer<Scene3D>                     _scene;
    Containers::Pointer<SceneGraph::DrawableGroup3D> _drawables;

    /* Camera helpers */
    Containers::Pointer<Object3D>             _objCamera3D;
    Containers::Pointer<SceneGraph::Camera3D> _camera3D;
    Containers::Pointer<ArcBallCamera>        _arcballCamera;

    Object3D*                           spheres[3];
    Containers::Pointer<GL::Mesh>       _mesh;
    Containers::Pointer<Shaders::Phong> _shader;

    bool _pausedMotion = false;
};

using namespace Math::Literals;

FEMSimulationExample::FEMSimulationExample(const Arguments& arguments) : Platform::Application{arguments, NoCreate} {
    /* Setup window */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Magnum Finite Element Method (FEM) Simulation Example")
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
        _objCamera3D.emplace(_scene.get());
        _objCamera3D->translate(Vector3::zAxis(3.0f));

        _camera3D.emplace(*_objCamera3D);
        _camera3D->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 4.0f / 3.0f, 0.001f, 1000.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());

        const Vector3 eye{ Vector3::zAxis(5.0f) };
        const Vector3 viewCenter{ };
        const Vector3 up{ Vector3::yAxis() };
        const Deg     fov = 45.0_degf;
        _arcballCamera.emplace(*_scene.get(), eye, viewCenter, up, fov, windowSize(), framebufferSize());
        _arcballCamera->setLagging(0.85f);
    }

    _mesh.emplace();
    *_mesh = MeshTools::compile(Primitives::icosphereSolid(3));
    _shader.emplace();

    /* Setup point (render as spheres) */ {
        spheres[0] = new Object3D(_scene.get());
        (new Icosphere(_mesh.get(), _shader.get(), 0xff0000_rgbf, spheres[0], _drawables.get()))
            ->translate(Vector3::yAxis(0.25f));
        (new Icosphere(_mesh.get(), _shader.get(), 0xff0000_rgbf, spheres[0], _drawables.get()))
            ->translate(Vector3::yAxis(0.25f))
            .rotateZ(120.0_degf);
        (new Icosphere(_mesh.get(), _shader.get(), 0xff0000_rgbf, spheres[0], _drawables.get()))
            ->translate(Vector3::yAxis(0.25f))
            .rotateZ(240.0_degf);

        spheres[1] = new Object3D(_scene.get());
        (new Icosphere(_mesh.get(), _shader.get(), 0x00ff00_rgbf, spheres[1], _drawables.get()))
            ->translate(Vector3::yAxis(0.50f));
        (new Icosphere(_mesh.get(), _shader.get(), 0x00ff00_rgbf, spheres[1], _drawables.get()))
            ->translate(Vector3::yAxis(0.50f))
            .rotateZ(120.0_degf);
        (new Icosphere(_mesh.get(), _shader.get(), 0x00ff00_rgbf, spheres[1], _drawables.get()))
            ->translate(Vector3::yAxis(0.50f))
            .rotateZ(240.0_degf);

        spheres[2] = new Object3D(_scene.get());
        (new Icosphere(_mesh.get(), _shader.get(), 0x0000ff_rgbf, spheres[2], _drawables.get()))
            ->translate(Vector3::yAxis(0.75f));
        (new Icosphere(_mesh.get(), _shader.get(), 0x0000ff_rgbf, spheres[2], _drawables.get()))
            ->translate(Vector3::yAxis(0.75f))
            .rotateZ(120.0_degf);
        (new Icosphere(_mesh.get(), _shader.get(), 0x0000ff_rgbf, spheres[2], _drawables.get()))
            ->translate(Vector3::yAxis(0.75f))
            .rotateZ(240.0_degf);
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    /* Start the timer, loop at 60 Hz max */
    setSwapInterval(1);
    setMinimalLoopPeriod(16);
}

void FEMSimulationExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    if(!_pausedMotion) {
        //
    }

    _arcballCamera->update();
    //    _camera3D->draw(*drawables);
    _arcballCamera->draw(*_drawables);

    swapBuffers();
    /* Run next frame immediately */
    redraw();
}

void FEMSimulationExample::viewportEvent(ViewportEvent& event) {
    /* Resize the main framebuffer */
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });
    _arcballCamera->reshape(event.windowSize(), event.framebufferSize());

    /* Recompute the camera's projection matrix */
    _camera3D->setViewport(event.framebufferSize());
}

void FEMSimulationExample::keyPressEvent(KeyEvent& event) {
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

void FEMSimulationExample::mousePressEvent(MouseEvent& event) {
    /* Enable mouse capture so the mouse can drag outside of the window */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_TRUE);

    _arcballCamera->initTransformation(event.position());

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void FEMSimulationExample::mouseReleaseEvent(MouseEvent&) {
    /* Disable mouse capture again */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_FALSE);
}

void FEMSimulationExample::mouseMoveEvent(MouseMoveEvent& event) {
    if(!event.buttons()) { return; }

    if(event.modifiers() & MouseMoveEvent::Modifier::Shift) {
        _arcballCamera->translate(event.position());
    } else { _arcballCamera->rotate(event.position()); }

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void FEMSimulationExample::mouseScrollEvent(MouseScrollEvent& event) {
    const Float delta = event.offset().y();
    if(Math::abs(delta) < 1.0e-2f) { return; }

    _arcballCamera->zoom(delta);

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}
} }

MAGNUM_APPLICATION_MAIN(Magnum::Examples::FEMSimulationExample)
