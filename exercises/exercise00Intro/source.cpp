#include <iostream>

#include <Core/EntryPoint.h>
#include <ATCG.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>

class Exercise00Layer : public atcg::Layer
{
public:
    Exercise00Layer(const std::string &name) : atcg::Layer(name) {}

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto &window = atcg::Application::get()->getWindow();

        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller = std::make_shared<atcg::CameraController>(aspect_ratio);

        mesh = atcg::IO::read_mesh("res/suzanne_blender.obj");
        mesh->uploadData();
    }

    // This gets called each frame
    virtual void onUpdate(float delta_time) override
    {
        camera_controller->onUpdate(delta_time);

        atcg::Renderer::clear();

        if (mesh && render_faces)
            atcg::Renderer::draw(mesh, atcg::ShaderManager::getShader("base"), camera_controller->getCamera());

        if (mesh && render_points)
            atcg::Renderer::drawPoints(mesh, glm::vec3(0), atcg::ShaderManager::getShader("flat"), camera_controller->getCamera());

        if (mesh && render_edges)
            atcg::Renderer::drawLines(mesh, glm::vec3(0), camera_controller->getCamera());
    }

    // Here, debug menus and guis can be created
    virtual void onImGuiRender() override
    {
        ImGui::BeginMainMenuBar();

        if (ImGui::BeginMenu("Exercise"))
        {
            ImGui::MenuItem("Show Render Settings", nullptr, &show_render_settings);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        if (show_render_settings)
        {
            ImGui::Begin("Render Settings");
            ImGui::Checkbox("Render Mesh:", &render_faces);
            ImGui::Checkbox("Render Edges:", &render_edges);
            ImGui::Checkbox("Render Vertices:", &render_points);
            ImGui::End();
        }
    }

    // This function is evaluated if an event (key, mouse, resize events, etc.) are triggered
    virtual void onEvent(atcg::Event &event) override
    {
        camera_controller->onEvent(event);

        atcg::EventDispatcher dispatcher(event);
        dispatcher.dispatch<atcg::FileDroppedEvent>(ATCG_BIND_EVENT_FN(Exercise00Layer::onFileDropped));
    }

    bool onFileDropped(atcg::FileDroppedEvent &event)
    {
        mesh = atcg::IO::read_mesh(event.getPath().c_str());
        mesh->uploadData();

        // Also reset camera
        const auto &window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller = std::make_shared<atcg::CameraController>(aspect_ratio);

        return true;
    }

private:
    bool show_render_settings = true;
    bool render_faces = true;
    bool render_points = false;
    bool render_edges = false;

    std::shared_ptr<atcg::CameraController> camera_controller;
    std::shared_ptr<atcg::Mesh> mesh;
};

class Exercise00 : public atcg::Application
{
public:
    Exercise00()
        : atcg::Application()
    {
        pushLayer(new Exercise00Layer("Layer"));
    }

    ~Exercise00() {}
};

// Entry Point (basically the main function)
atcg::Application *atcg::createApplication()
{
    return new Exercise00;
}