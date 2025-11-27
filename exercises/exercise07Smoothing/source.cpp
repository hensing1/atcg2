#include <iostream>

#include <Core/EntryPoint.h>
#include <ATCG.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <algorithm>
#include <queue>

#include <numeric>

using VertexHandle = atcg::Mesh::VertexHandle;
using EdgeHandle   = atcg::Mesh::EdgeHandle;

template<typename T>
struct LaplaceUniform
{
    atcg::Laplacian<T> calculate(const std::shared_ptr<atcg::Mesh>& mesh)
    {
        std::vector<Eigen::Triplet<T>> edge_weights;
        std::vector<Eigen::Triplet<T>> vertex_weights;
        size_t N = mesh->n_vertices();

        atcg::Laplacian<T> laplace;
        laplace.S.resize(N, N);
        laplace.M.resize(N, N);

        /// Exercise: - Calculate the weights for the S matrix
        ///           - edge_weights contains the triplet of 2 indices (i,j) and the value at this point
        ///           - You can use laplace.S.setFromTriplets(...) to set the entries of the matrix.
        ///             all entries with the same indices (i,j) will be added together.
        ///           - Calculate the weights for the M matrix

        // 

        return laplace;
    }
};

class Exercise07Layer : public atcg::Layer
{
public:
    Exercise07Layer(const std::string& name) : atcg::Layer(name) {}

    template<typename T, class LaplaceCalculator>
    void taubin_smoothing(const std::shared_ptr<atcg::Mesh>& mesh, LaplaceCalculator calculator)
    {
        atcg::Laplacian<T> laplacian = calculator.calculate(mesh);
        /// Exercise: - Calculate the Taubin operator

        // 

        // Convert OpenMesh representation to Eigen matrix
        Eigen::Matrix<T, -1, -1> v(mesh->n_vertices(), 3);
        for(auto v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            atcg::Mesh::Point p = mesh->point(*v_it);
            v(v_it->idx(), 0)   = p[0];
            v(v_it->idx(), 1)   = p[1];
            v(v_it->idx(), 2)   = p[2];
        }

        /// Exercise: Implement taubin update step

        // 

        // Convert Eigen matrix to OpenMesh
        for(auto v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            mesh->set_point(*v_it, atcg::Mesh::Point {v(v_it->idx(), 0), v(v_it->idx(), 1), v(v_it->idx(), 2)});
        }

        mesh->uploadData();
    }

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        mesh = atcg::IO::read_mesh("res/bunny.obj");
        mesh->uploadData();

        default_mesh = atcg::IO::read_mesh("res/bunny.obj");
        default_mesh->uploadData();
    }

    // This gets called each frame
    virtual void onUpdate(float delta_time) override
    {
        camera_controller->onUpdate(delta_time);

        atcg::Renderer::clear();

        if(mesh && render_faces)
            atcg::Renderer::draw(mesh, atcg::ShaderManager::getShader("base"), camera_controller->getCamera());

        if(mesh && render_points)
            atcg::Renderer::drawPoints(mesh,
                                       glm::vec3(0),
                                       atcg::ShaderManager::getShader("flat"),
                                       camera_controller->getCamera());

        if(mesh && render_edges) atcg::Renderer::drawLines(mesh, glm::vec3(0), camera_controller->getCamera());
    }

    virtual void onImGuiRender() override
    {
        ImGui::BeginMainMenuBar();

        if(ImGui::BeginMenu("Rendering"))
        {
            ImGui::MenuItem("Show Render Settings", nullptr, &show_render_settings);

            ImGui::MenuItem("Show Taubin Smoothing", nullptr, &show_taubin);

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        if(show_render_settings)
        {
            ImGui::Begin("Settings", &show_render_settings);

            ImGui::Checkbox("Render Vertices", &render_points);
            ImGui::Checkbox("Render Edges", &render_edges);
            ImGui::Checkbox("Render Mesh", &render_faces);
            ImGui::End();
        }

        if(show_taubin)
        {
            ImGui::Begin("Taubin Smoothing", &show_taubin);

            if(ImGui::Button("Iteration step"))
            {
                for(int i = 0; i < num_iterations_per_step; ++i)
                {
                    taubin_smoothing<float, LaplaceUniform<float>>(mesh, LaplaceUniform<float>());
                }
                iterations += num_iterations_per_step;
            }

            ImGui::InputInt("Iterations per step", &num_iterations_per_step);
            ImGui::Text(("Iterations: " + std::to_string(iterations)).c_str());

            if(show_smoothed)
            {
                if(ImGui::Button("View default"))
                {
                    show_smoothed = false;
                    std::swap(default_mesh, mesh);
                }
            }
            else
            {
                if(ImGui::Button("View smoothed"))
                {
                    show_smoothed = true;
                    std::swap(mesh, default_mesh);
                }
            }

            ImGui::End();
        }
    }

    // This function is evaluated if an event (key, mouse, resize events, etc.) are triggered
    virtual void onEvent(atcg::Event& event) override
    {
        camera_controller->onEvent(event);

        atcg::EventDispatcher dispatcher(event);
        dispatcher.dispatch<atcg::FileDroppedEvent>(ATCG_BIND_EVENT_FN(Exercise07Layer::onFileDropped));
    }

    bool onFileDropped(atcg::FileDroppedEvent& event)
    {
        mesh = atcg::IO::read_mesh(event.getPath().c_str());
        mesh->uploadData();

        default_mesh = atcg::IO::read_mesh(event.getPath().c_str());
        default_mesh->uploadData();

        // Also reset camera
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        return true;
    }

private:
    std::shared_ptr<atcg::CameraController> camera_controller;
    std::shared_ptr<atcg::Mesh> mesh;
    std::shared_ptr<atcg::Mesh> default_mesh;

    bool show_render_settings = false;
    bool render_faces         = true;
    bool render_points        = false;
    bool render_edges         = false;
    bool show_taubin          = true;
    bool show_smoothed        = true;

    float mu     = -0.9f;
    float lambda = 0.91f;

    int num_iterations_per_step = 1;
    int iterations              = 0;
};

class Exercise07 : public atcg::Application
{
public:
    Exercise07() : atcg::Application() { pushLayer(new Exercise07Layer("Layer")); }

    ~Exercise07() {}
};

atcg::Application* atcg::createApplication()
{
    return new Exercise07;
}