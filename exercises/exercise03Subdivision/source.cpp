#include "DataStructure/Mesh.h"
#include <iostream>

#include <Core/EntryPoint.h>
#include <ATCG.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <algorithm>

#include <math.h>

class Exercise03Layer : public atcg::Layer
{
public:
    Exercise03Layer(const std::string& name) : atcg::Layer(name) {}

    void subdivide_mesh(const std::shared_ptr<atcg::Mesh>& mesh)
    {
        // TODO: sqrt(3) subdivision
        mesh->request_edge_status();
        mesh->request_vertex_status();
        uint32_t num_features = mesh->find_feature_edges();
        std::cout << "Found " << num_features << " feature edges\n";

        size_t nv = mesh->n_vertices();
        size_t ne = mesh->n_edges();
        size_t nf = mesh->n_faces();

        auto eend = mesh->edges_end();
        auto fend = mesh->faces_end();

        auto new_pos_property = OpenMesh::makeTemporaryProperty<OpenMesh::VertexHandle, atcg::Mesh::Point>(*mesh.get());

        // Calculate new position of old vertices
        for(auto v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            /// Exercise: Please implement the new positions of the old vertices
            ///           - you can query for the valence of a vertex using:
            ///                   auto valence = (*v_it).valence();
            ///           - you can iterate over the vertices in the one-ring around a vertex v using:
            ///                         for (auto vv_it = v_it->vertices().begin(); vv_it != v_it->vertices().end();
            ///                         ++vv_it) {/*your code here*/}
            ///           - the property new_pos can be used to store the new positions of the old vertices:
            ///           new_pos_property[*v_it] = ...;
            ///           - the formula can be found on Slideset-Surface-RepresentationsI.pdf Slide 53 or in the
            ///           original paper:
            ///                          https://www.graphics.rwth-aachen.de/media/papers/sqrt31.pdf
            ///           - make sure to handle boundary (mesh->is_boundary) and feature (v_it->feature()) vertices
            ///           accordingly


            if(v_it->is_boundary())
            {
                std::vector<atcg::Mesh::VertexHandle> boundary_neighbors;
                for(auto vv_it = v_it->vertices().begin(); vv_it != v_it->vertices().end(); vv_it++)
                    if(vv_it->is_boundary()) boundary_neighbors.push_back(*vv_it);

                if(boundary_neighbors.size() != 2) continue;

                new_pos_property[*v_it] = mesh->point(*v_it) * 2 / 3 +
                                          (mesh->point(boundary_neighbors[0]) + mesh->point(boundary_neighbors[1])) / 6;
                continue;
            }

            uint valence           = v_it->valence();
            double neighbor_weight = (4 - 2 * std::cos(2 * M_PI / valence)) / (9 * valence);

            new_pos_property[*v_it] = (1 - valence * neighbor_weight) * mesh->point(*v_it);
            for(auto vv_it = v_it->vertices().begin(); vv_it != v_it->vertices().end(); vv_it++)
                new_pos_property[*v_it] += neighbor_weight * mesh->point(*vv_it);
        }

        // Set new vertex positions
        for(auto v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            if(mesh->is_boundary(*v_it) || v_it->feature()) continue;
            mesh->point(*v_it) = new_pos_property[*v_it];
        }

        // Split faces
        std::vector<atcg::Mesh::FaceHandle> faces;
        std::vector<atcg::Mesh::VertexHandle> centroids;
        for(auto f_it = mesh->faces_begin(); f_it != fend; ++f_it)
        {
            /// Exercise: Please implement the face splits
            ///           - you can iterate over the vertices of a face using:
            ///                   for (auto v_it = f_it->vertices().begin(); v_it != f_it->vertices().end(); ++v_it) {
            ///                   /*your code here*/ }
            ///           - split the faces at the centroid of each face (mesh->split)

            uint vertex_count = 0;
            auto midpoint     = atcg::Mesh::Point(0, 0, 0);
            for(auto v_it = f_it->vertices().begin(); v_it != f_it->vertices().end(); ++v_it)
            {
                midpoint += mesh->point(*v_it);
                vertex_count++;
            }
            if(vertex_count == 0) continue;
            midpoint /= vertex_count;

            mesh->split(*f_it, midpoint);
        }

        // Flip old edges
        for(auto e_it = mesh->edges_begin(); e_it != eend; ++e_it)
        {
            atcg::Mesh::EdgeHandle e = *e_it;

            /// Exercise: Please implement the edge flips
            ///           - make sure you check whether a flip is feasable with mesh->is_flip_ok(e)
            ///           - make sure you don't flip an edge which is a feature edge which can be checked with
            ///           e_it->feature

            if(!mesh->is_flip_ok(e) || e_it->feature()) continue;
            mesh->flip(e);
        }

        // Update rendering
        mesh->uploadData();
    }

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        mesh = atcg::IO::read_mesh("res/suzanne_blender.obj");
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
                                       atcg::ShaderManager::getShader("flat"),
                                       camera_controller->getCamera());

        if(mesh && render_edges) atcg::Renderer::drawLines(mesh, glm::vec3(0), camera_controller->getCamera());
    }

    virtual void onImGuiRender() override
    {
        update_grid = false;
        ImGui::BeginMainMenuBar();

        if(ImGui::BeginMenu("Rendering"))
        {
            ImGui::MenuItem("Show Render Settings", nullptr, &show_render_settings);
            ImGui::EndMenu();
        }

        if(ImGui::BeginMenu("Exercise"))
        {
            ImGui::MenuItem("Subdivision", nullptr, &show_subdivision);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        if(show_subdivision)
        {
            ImGui::Begin("Settings SD", &show_subdivision);

            if(ImGui::Button("Subdivide")) { subdivide_mesh(mesh); }

            ImGui::End();
        }

        if(show_render_settings)
        {
            ImGui::Begin("Settings", &show_render_settings);

            ImGui::Checkbox("Render Vertices", &render_points);
            ImGui::Checkbox("Render Edges", &render_edges);
            ImGui::Checkbox("Render Mesh", &render_faces);
            ImGui::Checkbox("Render Grid", &render_grid);
            ImGui::End();
        }
    }

    // This function is evaluated if an event (key, mouse, resize events, etc.) are triggered
    virtual void onEvent(atcg::Event& event) override
    {
        camera_controller->onEvent(event);

        atcg::EventDispatcher dispatcher(event);
        dispatcher.dispatch<atcg::FileDroppedEvent>(ATCG_BIND_EVENT_FN(Exercise03Layer::onFileDropped));
    }

    bool onFileDropped(atcg::FileDroppedEvent& event)
    {
        mesh = atcg::IO::read_mesh(event.getPath().c_str());

        mesh->uploadData();

        // Also reset camera
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        return true;
    }

private:
    std::shared_ptr<atcg::Mesh> mesh;
    std::shared_ptr<atcg::CameraController> camera_controller;

    bool show_render_settings = false;
    bool render_faces         = true;
    bool render_points        = false;
    bool render_edges         = false;
    bool update_grid          = false;
    bool render_grid          = true;
    float voxel_size          = 0.05f;

    bool show_subdivision = true;
};

class Exercise03 : public atcg::Application
{
public:
    Exercise03() : atcg::Application() { pushLayer(new Exercise03Layer("Layer")); }

    ~Exercise03() {}
};

atcg::Application* atcg::createApplication()
{
    return new Exercise03;
}
