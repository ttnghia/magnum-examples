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
#include <Magnum/GL/Version.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Math/Color.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Trade/MeshData.h>

#include "../motionblur/Icosphere.h"

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <Pez.h>
#include "SmokeSolver/SmokeSolver2D.h"

namespace Magnum { namespace Examples {
class SmokeSimulation2DExample : public Platform::Application {
public:
    explicit SmokeSimulation2DExample(const Arguments& arguments);

protected:
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void keyReleaseEvent(KeyEvent& event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;
    void textInputEvent(TextInputEvent& event) override;

    /* Window control */
    void showMenu();

    /* Simulation loop */
    void resetSimulation();

    bool _showMenu = true;
    ImGuiIntegration::Context _imGuiContext{ NoCreate };

    /* Scene and drawable group must be constructed before camera and other
       drawble objects */
    Containers::Pointer<Scene3D>                     _scene;
    Containers::Pointer<SceneGraph::DrawableGroup3D> _drawables;

    /* Camera helpers */
    Containers::Pointer<Object3D>             _objCamera3D;
    Containers::Pointer<SceneGraph::Camera3D> _camera3D;

    Containers::Pointer<SmokeSolver2D> _smokeSolver;

    bool _pausedMotion = false;
};

using namespace Math::Literals;

SmokeSimulation2DExample::SmokeSimulation2DExample(const Arguments& arguments) : Platform::Application{arguments, NoCreate} {
    /* Setup window */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Magnum Smoke Simulation 2D Example")
            .setSize(conf.size(), dpiScaling)
            .setWindowFlags(Configuration::WindowFlag::Resizable);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf)) {
            create(conf, glConf.setSampleCount(0));
        }
    }

    /* Setup ImGui, load a better font */
    {
        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        ImFontConfig fontConfig;
        fontConfig.FontDataOwnedByAtlas = false;
        const Vector2                     size = Vector2{ windowSize() } / dpiScaling();
        Utility::Resource                 rs{ "data" };
        Containers::ArrayView<const char> font = rs.getRaw("SourceSansPro-Regular.ttf");
        ImGui::GetIO().Fonts->AddFontFromMemoryTTF(
            const_cast<char*>(font.data()), Int(font.size()),
            16.0f * framebufferSize().x() / size.x(), &fontConfig);

        _imGuiContext = ImGuiIntegration::Context{ *ImGui::GetCurrentContext(),
                                                   Vector2{ windowSize() } / dpiScaling(), windowSize(), framebufferSize() };

        /* Setup proper blending to be used by ImGui */
        GL::Renderer::setBlendFunction(
            GL::Renderer::BlendFunction::SourceAlpha,
            GL::Renderer::BlendFunction::OneMinusSourceAlpha);
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
        // _arcballCamera.emplace(*_scene.get(), eye, viewCenter, up, fov, windowSize(), framebufferSize());
        // _arcballCamera->setLagging(0.85f);
    }

    /* Explicitly disable depth test */
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

    _smokeSolver.emplace();
    _smokeSolver->PezInitialize();
    //GL::Renderer::setClearColor(Color4(1, 1, 1, 1));
    // GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth)
    //     .bind();
    {
        /* Entering a section with 3rd-party OpenGL code -- clean up all state that
           could cause accidental modifications of our objects from outside */
        //        GL::Context::current().resetState(GL::Context::State::EnterExternal);

        /* Raw OpenGL calls */
        // PezInitialize(PEZ_VIEWPORT_WIDTH, PEZ_VIEWPORT_HEIGHT);
        // ...

        /* Exiting a section with 3rd-party OpenGL code -- reset our state tracker */
        //        GL::Context::current().resetState(GL::Context::State::ExitExternal);
    }

    /* Start the timer, loop at 60 Hz max */
    setSwapInterval(1);
    setMinimalLoopPeriod(16);
}

void SmokeSimulation2DExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth)
        .bind();
    _imGuiContext.newFrame();

    /* Enable text input, if needed */
    if(ImGui::GetIO().WantTextInput && !isTextInputActive()) {
        startTextInput();
    } else if(!ImGui::GetIO().WantTextInput && isTextInputActive()) {
        stopTextInput();
    }

    if(!_pausedMotion) {
        //
    }

    _smokeSolver->PezUpdate();
    _smokeSolver->PezRender(0);

    /* Menu for parameters */
    if(_showMenu) { showMenu(); }

    /* Update application cursor */
    _imGuiContext.updateApplicationCursor(*this);

    /* Render ImGui window */
    {
        GL::Renderer::enable(GL::Renderer::Feature::Blending);
        GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
        GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);

        //        _imGuiContext.drawFrame();

        GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
        GL::Renderer::disable(GL::Renderer::Feature::Blending);
    }

    swapBuffers();
    /* Run next frame immediately */
    redraw();
}

void SmokeSimulation2DExample::viewportEvent(ViewportEvent& event) {
    /* Resize the main framebuffer */
    GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });

    /* Relayout ImGui */
    _imGuiContext.relayout(Vector2{ event.windowSize() } / event.dpiScaling(), event.windowSize(), event.framebufferSize());

    /* Recompute the camera's projection matrix */
    _camera3D->setViewport(event.framebufferSize());
}

void SmokeSimulation2DExample::keyPressEvent(KeyEvent& event) {
    switch(event.key()) {
        case KeyEvent::Key::R:
            event.setAccepted(true);
            break;
        case KeyEvent::Key::Space:
            _pausedMotion ^= true;
            event.setAccepted(true);
            break;
        default:
            if(_imGuiContext.handleKeyPressEvent(event)) {
                event.setAccepted(true);
            }
    }
}

void SmokeSimulation2DExample::keyReleaseEvent(KeyEvent& event) {
    if(_imGuiContext.handleKeyReleaseEvent(event)) {
        event.setAccepted(true);
        return;
    }
}

void SmokeSimulation2DExample::mousePressEvent(MouseEvent& event) {
    /* Enable mouse capture so the mouse can drag outside of the window */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_TRUE);

    if(_imGuiContext.handleMousePressEvent(event)) {
        event.setAccepted(true);
        return;
    }

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void SmokeSimulation2DExample::mouseReleaseEvent(MouseEvent& event) {
    /* Disable mouse capture again */
    /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
    SDL_CaptureMouse(SDL_FALSE);

    if(_imGuiContext.handleMouseReleaseEvent(event)) {
        event.setAccepted(true);
    }
}

void SmokeSimulation2DExample::mouseMoveEvent(MouseMoveEvent& event) {
    if(_imGuiContext.handleMouseMoveEvent(event)) {
        event.setAccepted(true);
        return;
    }
}

void SmokeSimulation2DExample::mouseScrollEvent(MouseScrollEvent& event) {
    const Float delta = event.offset().y();
    if(Math::abs(delta) < 1.0e-2f) { return; }

    if(_imGuiContext.handleMouseScrollEvent(event)) {
        /* Prevent scrolling the page */
        event.setAccepted();
        return;
    }
}

void SmokeSimulation2DExample::textInputEvent(TextInputEvent& event) {
    if(_imGuiContext.handleTextInputEvent(event)) {
        event.setAccepted(true);
    }
}

void SmokeSimulation2DExample::showMenu() {
    ImGui::SetNextWindowPos({ 10.0f, 10.0f }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("Options", nullptr);

#if 0
    /* General information */
    ImGui::Text("Hide/show menu: H");
    ImGui::Text("Num. particles: %d",   Int(_fluidSolver->numParticles()));
    ImGui::Text("Rendering: %3.2f FPS", Double(ImGui::GetIO().Framerate));
    ImGui::Spacing();

    /* Rendering parameters */
    if(ImGui::TreeNode("Particle Rendering")) {
        ImGui::PushID("Particle Rendering");
        {
            constexpr const char* items[]   = { "Uniform", "Ramp by ID" };
            static Int            colorMode = 1;
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
            if(ImGui::Combo("Color Mode", &colorMode, items, 2)) {
                _drawableParticles->setColorMode(ParticleSphereShader2D::ColorMode(colorMode));
            }
            ImGui::PopItemWidth();
            if(colorMode == 0) { /* Uniform color */
                static Color3 color = _drawableParticles->color();
                if(ImGui::ColorEdit3("Color", color.data())) {
                    _drawableParticles->setColor(color);
                }
            }
        }
        ImGui::PopID();
        ImGui::TreePop();
    }
    ImGui::Spacing();
    ImGui::Separator();

    /* Simulation parameters */
    if(ImGui::TreeNodeEx("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Simulation");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
        ImGui::InputFloat("Speed", &_speed);
        ImGui::Checkbox("Auto emit particles 5 times", &_bAutoEmitParticles);
        ImGui::PopItemWidth();
        ImGui::BeginGroup();
        ImGui::Checkbox("Mouse interaction", &_bMouseInteraction);
        if(_bMouseInteraction) {
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
            ImGui::SliderFloat("Radius",    &_mouseInteractionRadius,    1.0f, 10.0f);
            ImGui::SliderFloat("Magnitude", &_mouseInteractionMagnitude, 1.0f, 10.0f);
            ImGui::PopItemWidth();
        }
        ImGui::EndGroup();
        ImGui::PopID();
        ImGui::TreePop();
    }
    ImGui::Spacing();
    ImGui::Separator();

    /* Reset */
    ImGui::Spacing();
    if(ImGui::Button("Emit Particles")) {
        _fluidSolver->emitParticles();
    }
    ImGui::SameLine();
    if(ImGui::Button(_pausedSimulation ? "Play Sim" : "Pause Sim")) {
        _pausedSimulation ^= true;
    }
    ImGui::SameLine();
#endif
    if(ImGui::Button("Reset Sim")) {
        resetSimulation();
    }

    ImGui::End();
}

void SmokeSimulation2DExample::resetSimulation() {
    // _fluidSolver->reset();
    // _pausedSimulation = false;
    // _evolvedTime      = 0.0f;
    // _numEmission      = 0;
}
} }

MAGNUM_APPLICATION_MAIN(Magnum::Examples::SmokeSimulation2DExample)
