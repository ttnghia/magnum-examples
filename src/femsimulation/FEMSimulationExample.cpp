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

#include <Corrade/Containers/Pointer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>

#include "Application/ImGuiApplication.h"
#include "Simulation/Mesh.h"
#include "Simulation/Simulator.h"

class Application : public ImGuiApplication {
public:
    explicit Application(const Arguments& arguments);

private:
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void keyReleaseEvent(KeyEvent& event) override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;
    void textInputEvent(TextInputEvent& event) override;

    void beginFrame();
    void endFrame();

    void showMenuHeader();
    void showMenuFooter(bool bButtonResetCamera = true);

    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    void setupCamera();

    /* Window control */
    bool   m_bVsync { true };
    Color3 m_BkgColor { 0.35f };

    /* Scene and drawable group */
    Scene3D                     m_Scene;
    SceneGraph::DrawableGroup3D m_Drawables;

    /* Ground grid */
    Containers::Pointer<Grid> m_Grid;

    /* Camera helpers */
    Vector3 m_DefaultCamPosition { 0.0f, 1.5f, 8.0f };
    Vector3 m_DefaultCamTarget { 0.0f, 1.0f, 0.0f };
    Containers::Pointer<ArcBallCamera> m_Camera;

    /* Window control */
    bool m_bShowMenu { true };
    ImGuiIntegration::Context m_ImGuiContext{ NoCreate };

    void drawEvent() override;
    void keyPressEvent(KeyEvent& event) override;
    void showMenu();

    void resetSimulation();
    void initializeSagFree();

    Containers::Pointer<Simulator> m_simulator;
    Timer  m_timer;
    String m_status { "Status: Paused" };
    bool   m_pause { true };

    /* The simulated mesh */
    Containers::Pointer<TetMesh> m_mesh;

    /* For plotting frame simulation time */
    static inline constexpr u64 sMaxFrames { 30 };
    float  m_frameTime[sMaxFrames];
    float  m_lastFrameTime{ 0 };
    size_t m_offset { 0 };
};

/****************************************************************************************************/
ImGuiApplication::ImGuiApplication(const std::string& title, const Arguments& arguments,
                                   const Vector2i& defaultWindowSize) :
    GLApplication{title, arguments, defaultWindowSize} {
    /* Setup window */
    create(Configuration{}
               .setTitle(title)
               .setSize(defaultWindowSize)
               .setWindowFlags(Configuration::WindowFlag::Resizable),
           GLConfiguration{}.setSampleCount(this->dpiScaling({}).max() < 2.0f ? 8 : 2));
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::setClearColor(m_BkgColor);
    setSwapInterval(1);

    /* Move the window to center */
    GLFWmonitor* const monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode    = glfwGetVideoMode(monitor);
    CORRADE_INTERNAL_ASSERT(mode != nullptr);
    int monitorX, monitorY;
    glfwGetMonitorPos(monitor, &monitorX, &monitorY);
    glfwSetWindowPos(window(),
                     monitorX + (mode->width - defaultWindowSize.x()) / 2,
                     monitorY + (mode->height - defaultWindowSize.y()) / 2);

    /* Setup scene objects and camera */
    m_Grid.emplace(&m_Scene, &m_Drawables);

    /* Configure camera */
    setupCamera();

    /* Setup ImGui and ImGuizmo */
    m_ImGuiContext = ImGuiIntegration::Context(Vector2{ windowSize() } / dpiScaling(),
                                               windowSize(), framebufferSize());
    ImGui::StyleColorsDark();

    /* Setup proper blending to be used by ImGui. There's a great chance
       you'll need this exact behavior for the rest of your scene. If not, set
       this only for the drawFrame() call. */
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);
}

/****************************************************************************************************/
void GLApplication::viewportEvent(ViewportEvent& event) {
    const auto newBufferSize = event.framebufferSize();

    /* Resize the main framebuffer */
    GL::defaultFramebuffer.setViewport({ {}, newBufferSize });

    /* Resize camera */
    m_Camera->reshape(event.windowSize(), event.framebufferSize());
}

/****************************************************************************************************/
void GLApplication::keyPressEvent(KeyEvent& event) {
    switch(event.key()) {
        case  KeyEvent::Key::V:
            m_bVsync ^= true;
            setSwapInterval(m_bVsync);
            event.setAccepted(true);
            break;
        case KeyEvent::Key::S:
            if(m_Camera->lagging() > 0.0f) {
                Debug{} << "Disable camera smooth navigation";
                m_Camera->setLagging(0.0f);
            } else {
                Debug{} << "Enable camera smooth navigation";
                m_Camera->setLagging(0.85f);
            }
            break;
        case KeyEvent::Key::R:
            m_Camera->reset();
            break;
        case KeyEvent::Key::Esc:
            exit(0);
    }
}

/****************************************************************************************************/
void GLApplication::mousePressEvent(MouseEvent& event) {
    m_Camera->initTransformation(event.position());
}

/****************************************************************************************************/
void GLApplication::mouseMoveEvent(MouseMoveEvent& event) {
    if(!event.buttons()) { return; }
    if(event.buttons() & MouseMoveEvent::Button::Left) {
        m_Camera->rotate(event.position());
    } else {
        m_Camera->translate(event.position());
    }
    event.setAccepted();
}

/****************************************************************************************************/
void GLApplication::mouseScrollEvent(MouseScrollEvent& event) {
    const float delta = event.offset().y();
    if(std::abs(delta) < 1.0e-2f) {
        return;
    }
    m_Camera->zoom(delta);
    event.setAccepted();
}

/****************************************************************************************************/
void GLApplication::setupCamera() {
    m_Camera.emplace(m_Scene, m_DefaultCamPosition, m_DefaultCamTarget, Vector3::yAxis(),
                     45.0_degf, windowSize(), framebufferSize());
    m_Camera->setLagging(0.85f);
}

/****************************************************************************************************/
void ImGuiApplication::viewportEvent(ViewportEvent& event) {
    GLApplication::viewportEvent(event);

    /* Relayout ImGui */
    m_ImGuiContext.relayout(Vector2{ event.windowSize() } / event.dpiScaling(),
                            event.windowSize(), event.framebufferSize());
}

/****************************************************************************************************/
void ImGuiApplication::keyPressEvent(KeyEvent& event) {
    if(m_ImGuiContext.handleKeyPressEvent(event)) {
        event.setAccepted(true);
    } else {
        GLApplication::keyPressEvent(event);
        if(!event.isAccepted()) {
            if(event.key() == KeyEvent::Key::H) {
                m_bShowMenu ^= true;
                event.setAccepted(true);
            }
        }
    }
}

void ImGuiApplication::keyReleaseEvent(KeyEvent& event) {
    if(m_ImGuiContext.handleKeyReleaseEvent(event)) {
        event.setAccepted(true);
    }
}

/****************************************************************************************************/
void ImGuiApplication::mousePressEvent(MouseEvent& event) {
    if(m_ImGuiContext.handleMousePressEvent(event)) {
        event.setAccepted(true);
    } else {
        GLApplication::mousePressEvent(event);
    }
}

/****************************************************************************************************/
void ImGuiApplication::mouseReleaseEvent(MouseEvent& event) {
    if(m_ImGuiContext.handleMouseReleaseEvent(event)) {
        event.setAccepted(true);
    }
}

/****************************************************************************************************/
void ImGuiApplication::mouseMoveEvent(MouseMoveEvent& event) {
    if(m_ImGuiContext.handleMouseMoveEvent(event)) {
        event.setAccepted(true);
    } else {
        GLApplication::mouseMoveEvent(event);
    }
}

/****************************************************************************************************/
void ImGuiApplication::mouseScrollEvent(MouseScrollEvent& event) {
    if(m_ImGuiContext.handleMouseScrollEvent(event)) {
        /* Prevent scrolling the page */
        event.setAccepted(true);
    } else {
        GLApplication::mouseScrollEvent(event);
    }
}

/****************************************************************************************************/
void ImGuiApplication::textInputEvent(TextInputEvent& event) {
    if(m_ImGuiContext.handleTextInputEvent(event)) {
        event.setAccepted(true);
    }
}

/****************************************************************************************************/
void ImGuiApplication::beginFrame() {
    m_ImGuiContext.newFrame();
    /* Enable text input, if needed */
    if(ImGui::GetIO().WantTextInput && !isTextInputActive()) {
        startTextInput();
    } else if(!ImGui::GetIO().WantTextInput && isTextInputActive()) {
        stopTextInput();
    }
}

/****************************************************************************************************/
void ImGuiApplication::endFrame() {
    /* Update application cursor */
    m_ImGuiContext.updateApplicationCursor(*this);

    /* Set appropriate states. If you only draw imgui UI, it is sufficient to do this once in the constructor. */
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);

    m_ImGuiContext.drawFrame();

    /* Reset state. Only needed if you want to draw something else with different state next frame. */
    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);
}

/****************************************************************************************************/
void ImGuiApplication::showMenuHeader() {
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("Options", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Hide/show menu: H | Exit: ESC");
    ImGui::Text("%3.2f FPS", static_cast<double>(ImGui::GetIO().Framerate));
    ImGui::SameLine(100);
    if(ImGui::Checkbox("VSync", &m_bVsync)) {
        setSwapInterval(m_bVsync);
    }
    ImGui::Spacing();
    ImGui::Checkbox("Render grid", &m_Grid->enabled());
    if(ImGui::ColorEdit3("Background color", m_BkgColor.data())) {
        GL::Renderer::setClearColor(m_BkgColor);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
}

void ImGuiApplication::showMenuFooter(bool bButtonResetCamera) {
    if(bButtonResetCamera) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        if(ImGui::Button("Reset camera")) {
            m_Camera->reset();
        }
    }
    ImGui::End();
}

Application::Application(const Arguments& arguments) :
    ImGuiApplication{"Sag-Free Initialization for Deformable Simulation", arguments} {
    /* Change camera */
    m_DefaultCamPosition = Vector3(-40, 20, 65);
    m_DefaultCamTarget   = Vector3(-10, -7, 0);
    setupCamera();

    /* Change grid */
    m_Grid->transform(Matrix4::translation(Vector3{ 0, -30, -100 }) * Matrix4::scaling(Vector3{ 30 }));

    /* Setup mesh and simulator */
    m_mesh.reset(new TetMesh("Data/longbar.mesh"));
    m_simulator.reset(new Simulator(m_mesh.get()));
}

void Application::drawEvent() {
    /* Run the simulation 1 step */
    if(!m_pause) {
        m_timer.tick();
        m_simulator->advanceStep();
        m_timer.tock();
        static u32 count { 0 };
        ++count;
        if(count == 10) {
            m_frameTime[m_offset] = m_timer.getLastRunTime();
            m_lastFrameTime       = m_frameTime[m_offset];
            m_offset = (m_offset + 1) % sMaxFrames;
            count    = 0;
        }
        char buff[128];
        sprintf(buff, "Status: Running simulation (t = %.2f (s))", m_simulator->m_generalParams.time);
        m_status = String(buff);
    }

    /* Draw everything */
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    ImGuiApplication::beginFrame();
    m_Camera->update();
    m_Camera->draw(m_Drawables); /* draw grid */
    m_mesh->draw(m_Camera.get(), Vector2{ framebufferSize() });
    if(m_bShowMenu) {
        showMenuHeader();
        showMenu();
        showMenuFooter();
    }
    ImGuiApplication::endFrame();
    swapBuffers();
    redraw();
}

void Application::keyPressEvent(KeyEvent& event) {
    ImGuiApplication::keyPressEvent(event);
    if(event.isAccepted()) {
        return;
    }
    switch(event.key()) {
        case KeyEvent::Key::Space:
            m_pause ^= true;
            if(m_pause) { m_status = "Status: Paused"; }
            break;
        case KeyEvent::Key::F5:
            resetSimulation();
            break;
        case KeyEvent::Key::F8:
            if(!m_pause) {
                m_status = "Error: Cannot run sag-free init. after started simulation\n"
                           "(please reset the simulation first)";
            } else {
                initializeSagFree();
            }
            break;
        default:;
    }
}

void Application::showMenu() {
    ImGui::PushItemWidth(120);
    if(ImGui::CollapsingHeader("General Simulation Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("General-Parameters");
        ImGui::InputFloat("Gravity", &m_simulator->m_generalParams.gravity[1]);
        ImGui::InputFloat("Damping", &m_simulator->m_generalParams.damping);
        if(ImGui::InputFloat("Attachement Stiffness", &m_simulator->m_generalParams.attachmentStiffness)) {
            m_simulator->updateConstraintParameters();
        }
        ImGui::Spacing();
        ImGui::InputFloat("Frame Time", &m_simulator->m_generalParams.dt, 0.0f, 0.0f, "%.6g");
        ImGui::InputInt("Substeps", &m_simulator->m_generalParams.subSteps);
        ImGui::PopID();
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if(ImGui::CollapsingHeader("Wind", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Wind");
        ImGui::Checkbox("Enable", &m_simulator->m_wind.enable);
        ImGui::InputFloat("Time enable", &m_simulator->m_wind.timeEnable);
        ImGui::InputFloat("Magnitude",   &m_simulator->m_wind.magnitude);
        ImGui::InputFloat("Frequency",   &m_simulator->m_wind.frequency);
        ImGui::PopID();
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if(ImGui::CollapsingHeader("FEM Material", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("FEM-Material");
        const char* items[] = { "Corotational", "StVK", "NeoHookean-ExtendLog" };
        ImGui::PopItemWidth();
        if(ImGui::Combo("Material", &m_simulator->m_FEMMaterial.type, items, IM_ARRAYSIZE(items))) {
            m_simulator->updateConstraintParameters();
        }
        ImGui::PushItemWidth(120);
        if(ImGui::InputFloat("mu", &m_simulator->m_FEMMaterial.mu)
           || ImGui::InputFloat("lambda", &m_simulator->m_FEMMaterial.lambda)
           || (m_simulator->m_FEMMaterial.type == FEMConstraint::Material::StVK
               && ImGui::InputFloat("kappa", &m_simulator->m_FEMMaterial.kappa))) {
            m_simulator->updateConstraintParameters();
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::PopItemWidth();

    if(ImGui::CollapsingHeader("Timing", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Timing");
        char buff[32];
        sprintf(buff, "Frame Time:\n%5.2f (ms)", m_lastFrameTime);
        float minVal = 1e10;
        float maxVal = -1e10;
        for(size_t i = 0; i < sMaxFrames; ++i) {
            if(minVal > m_frameTime[i]) { minVal = m_frameTime[i]; }
            if(maxVal < m_frameTime[i]) { maxVal = m_frameTime[i]; }
        }
        ImGui::PlotLines(buff, m_frameTime, IM_ARRAYSIZE(m_frameTime), m_offset, "",
                         minVal - 0.1f, maxVal + 0.1f, ImVec2(0, 80.0f));
        ImGui::PopID();
    }
    ImGui::Text(m_status.c_str());
    ImGui::Spacing();
    if(ImGui::Button("Initialize Sag-Free")) {
        m_simulator->initializeSagFreeSimulation();
    }
    if(ImGui::Button("Pause/Resume")) {
        m_pause ^= true;
        if(m_pause) { m_status = "Status: Paused"; }
    }
    ImGui::SameLine();
    if(ImGui::Button("Reset")) {
        resetSimulation();
    }
}

void Application::resetSimulation() {
    m_pause  = true;
    m_status = "Status: Paused";
    m_simulator->reset();
}

MAGNUM_APPLICATION_MAIN(Application)
