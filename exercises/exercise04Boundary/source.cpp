#include <iostream>

#include <Core/EntryPoint.h>
#include <ATCG.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <algorithm>


class Exercise04Layer : public atcg::Layer
{
public:
    Exercise04Layer(const std::string& name) : atcg::Layer(name) {}

    Eigen::SparseMatrix<int> compute_boundary_operator(const std::shared_ptr<atcg::Mesh>& mesh)
    {
        /// Exercise: -Implement the \partial_2 boundary operator of the given mesh
        ///           -Define a fixed order for the edges
        ///           -You can use Eigen::SparseMatrix<int>::setFromTriplets to build the sparse matrix
        ///           -You can use eh.idx() to get a unique index for each edge (likewise for faces and vertices)

        // 

        // Dummy solution
        return Eigen::SparseMatrix<int>();
    }

    Eigen::VectorXi compute_boundary_edges(const Eigen::SparseMatrix<int>& M)
    {
        /// Exercise: -Get a vector with n_edges() entries where non-boundary edges have value 0
        ///           -OpenMesh of course has methods to get boundary edges directly. DO NOT use those.

        // 

        // Dummy solution
        return Eigen::VectorXi();
    }

    void color_mesh(const std::shared_ptr<atcg::Mesh>& mesh, const Eigen::VectorXi& sum)
    {
        /// Exercise: -Color all vertices that make up the boundary edges red
        ///           -This framework does not support to color in edges directly, so you have to color in the
        ///           respective vertices -sum contains the output of 'compute_boundary_edges'.
        ///           - Again, DO NOT use OpenMesh functions

        // 
    }

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        mesh = atcg::IO::read_mesh("res/maxear.obj");
        mesh->request_vertex_colors();
        for(auto vt: mesh->vertices()) { mesh->set_color(vt, atcg::Mesh::Color(255)); }

        atcg::normalize(mesh);

        Eigen::SparseMatrix<int> M = compute_boundary_operator(mesh);
        Eigen::VectorXi sum        = compute_boundary_edges(M);
        color_mesh(mesh, sum);

        mesh->uploadData();
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
                                       atcg::ShaderManager::getShader("base"),
                                       camera_controller->getCamera());

        if(mesh && render_edges) atcg::Renderer::drawLines(mesh, glm::vec3(0), camera_controller->getCamera());
    }

    virtual void onImGuiRender() override
    {
        ImGui::BeginMainMenuBar();

        if(ImGui::BeginMenu("Rendering"))
        {
            ImGui::MenuItem("Show Render Settings", nullptr, &show_render_settings);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        if(show_render_settings)
        {
            ImGui::Begin("Settings", &show_render_settings);

            ImGui::Checkbox("Render Faces", &render_faces);
            ImGui::Checkbox("Render Edges", &render_edges);
            ImGui::Checkbox("Render Points", &render_points);

            ImGui::End();
        }
    }

    // This function is evaluated if an event (key, mouse, resize events, etc.) are triggered
    virtual void onEvent(atcg::Event& event) override { camera_controller->onEvent(event); }

private:
    std::shared_ptr<atcg::Mesh> mesh;
    std::shared_ptr<atcg::CameraController> camera_controller;

    bool show_render_settings = false;
    bool render_faces         = true;
    bool render_edges         = true;
    bool render_points        = false;
};

class Exercise04 : public atcg::Application
{
public:
    Exercise04() : atcg::Application() { pushLayer(new Exercise04Layer("Layer")); }

    ~Exercise04() {}
};

atcg::Application* atcg::createApplication()
{
    return new Exercise04;
}