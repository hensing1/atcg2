#include <iostream>

#include <Core/EntryPoint.h>
#include <ATCG.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <algorithm>

#include "MarchingCubesTable.h"

struct SDFVoxel
{
    float sdf;
    glm::vec3 color;
};

using SDFGrid = atcg::Grid<SDFVoxel>;

class Exercise05Layer : public atcg::Layer
{
public:
    Exercise05Layer(const std::string& name) : atcg::Layer(name) {}

    struct SDFResult
    {
        float sdf;
        glm::vec3 color;
    };

    struct SDF
    {
        virtual SDFResult operator()(const glm::vec3& p) = 0;
    };

    struct SDFHeart : public SDF
    {
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            float x = p.x;
            float y = p.y;
            float z = p.z;
            return {std::pow(x * x + 9.f / 4.f * y * y + z * z - 1.f, 3.f) - x * x * z * z * z -
                        9.f / 80.f * y * y * z * z * z,
                    glm::vec3(0, 1, 0)};
        }
    };

    struct SDFSphere : public SDF
    {
        glm::vec3 position;
        float radius;
        glm::vec3 color;

        SDFSphere(const glm::vec3& position, float radius, const glm::vec3& color = glm::vec3(1))
            : position(position),
              radius(radius),
              color(color)
        {
        }

        /// Exercise: Implement the signed distance function of a sphere
        ///           - Member variables:
        ///             * position - Position of the sphere
        ///             * radius - Radius of the sphere
        ///             * color - Color of the sphere
        ///           - This function should return an SDFResult with the sdf value evaluated at p and the color of the
        ///           sphere
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    struct SDFBox : public SDF
    {
        glm::vec3 position;
        glm::vec3 bound;
        glm::vec3 color;

        SDFBox(const glm::vec3& position, const glm::vec3& bound, const glm::vec3& color = glm::vec3(1))
            : position(position),
              bound(bound),
              color(color)
        {
        }

        float vmax(const glm::vec3& p) { return std::max(std::max(p.x, p.y), p.z); }

        /// Exercise: Implement the signed distance function of a Box
        ///           - Member variables:
        ///             * position - Center position of the box
        ///             * bound - The side length of the box in each dimension
        ///             * color - Color of the box
        ///           - This function should return an SDFResult with the sdf value evaluated at p and the color of the
        ///           box
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    struct SDFCylinder : public SDF
    {
        glm::vec3 position;
        float radius;
        uint32_t axis;
        glm::vec3 color;

        SDFCylinder(const glm::vec3& position, float radius, uint32_t axis, const glm::vec3& color = glm::vec3(1))
            : position(position),
              radius(radius),
              axis(axis),
              color(color)
        {
        }

        /// Exercise: Implement the signed distance function of an axis-aligned cylinder
        ///           - Member variables:
        ///             * position - Center position of the box
        ///             * radius - The radius of the cylinder
        ///             * axis - On which axis the cylinder should be placed
        ///             * color - The color of the cylinder
        ///           - This function should return an SDFResult with the sdf value evaluated at p and the color of the
        ///           Cylinder
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    struct SDFUnion : public SDF
    {
        SDF *sdf1, *sdf2;
        SDFUnion(SDF* sdf1, SDF* sdf2) : sdf1(sdf1), sdf2(sdf2) {}

        /// Exercise: Implement the Union operator of two SDFs
        ///           - Member variables:
        ///             * sdf1 - The first sdf
        ///             * sdf2 - The second sdf
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    struct SDFIntersection : public SDF
    {
        SDF *sdf1, *sdf2;
        SDFIntersection(SDF* sdf1, SDF* sdf2) : sdf1(sdf1), sdf2(sdf2) {}

        /// Exercise: Implement the Intersection operator of two SDFs
        ///           - Member variables:
        ///             * sdf1 - The first sdf
        ///             * sdf2 - The second sdf
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    struct SDFDifference : public SDF
    {
        SDF *sdf1, *sdf2;
        SDFDifference(SDF* sdf1, SDF* sdf2) : sdf1(sdf1), sdf2(sdf2) {}

        /// Exercise: Implement the Difference operator of two SDFs
        ///           - Member variables:
        ///             * sdf1 - The first sdf
        ///             * sdf2 - The second sdf
        virtual SDFResult operator()(const glm::vec3& p) override
        {
            // 

            return {0, glm::vec3(0)};
        }
    };

    void fillGrid(const std::shared_ptr<SDFGrid>& grid, SDF* sdf)
    {
        for(uint32_t i = 0; i < grid->voxels_per_volume(); ++i)
        {
            glm::ivec3 voxel = grid->index2voxel(i);
            glm::vec3 p      = grid->voxel2position(voxel);

            SDFResult res    = (*sdf)(p);
            (*grid)[i].sdf   = res.sdf;
            (*grid)[i].color = res.color;
        }
    }

    inline glm::vec3 interpolate_point(const float isovalue,
                                       const glm::vec3 vector_1,
                                       const float sdf_1,
                                       const glm::vec3 vector_2,
                                       const float sdf_2)
    {
        /// Exercise: Implement sdf interpolation
        ///           - Be aware of potential divisions by 0

        // 

        return (vector_1 + vector_2) / 2.0f;
    }

    void marching_cubes(const std::shared_ptr<SDFGrid>& grid, const std::shared_ptr<atcg::Mesh>& mesh)
    {
        mesh->request_vertex_colors();
        for(uint32_t index = 0; index < grid->voxels_per_volume(); ++index)
        {
            glm::ivec3 voxel      = grid->index2voxel(index);
            glm::vec3 voxel_start = grid->voxel2position(voxel);

            const float I = grid->voxel_side_length();
            const float O = 0.0f;

            // Get neighboring voxel positions
            /// Exercise: Calculate the position of neighboring voxels based on the voxel_start-
            ///           - The ordering of the points has to be the following, where index 0 = voxel_start
            ///             and the other points should have a distance of grid->voxel_side_length() in each dimension.
            ///
            ///                            7------------6
            ///                           /|           /|
            ///                          / |          / |
            ///                         3------------2  |
            ///                         |  |         |  |
            ///                         |  4---------|--5
            ///                         | /          | /
            ///                         |/           |/
            ///                         0------------1
            ///
            ///
            glm::vec3 voxel_positions[8] = {
                glm::vec3(voxel_start.x + O, voxel_start.y + O, voxel_start.z + O),
                glm::vec3(0),
                glm::vec3(0),
                glm::vec3(0),
                glm::vec3(0),
                glm::vec3(0),
                glm::vec3(0),
                glm::vec3(0),
            };

            // 

            // Get voxel information
            float sdf_values[8];
            glm::vec3 color_values[8];
            bool all_valid = true;

            /// Exercise: Read out the sdf and color information of neighboring voxels
            ///           -A voxel is only valid if it is inside the medium (grid->insideVolume(...))
            ///           -If at least one voxel is invalid, we cannot generate any surface
            ///           -You can read the voxel at position p via (*grid)(p)
            // 

            if(!all_valid) continue;

            const float ISOVALUE = 0.0f;

#define SET_BIT(ind, i) ind |= (1 << i)

            uint8_t cubeindex = 0;
            /// Exercse: Calculate the cube index by encoding the sign of each neighboring voxel as one bit
            ///          -Use the SET_BIT makro to change the corresponding bit
            ///          -If the shading of your result is wrong, try inverting the criteria

            // 

#undef SET_BIT

            // No edges -> early out
            if(edge_table[cubeindex] == 0) continue;

            glm::vec3 vertex_list[12];
            atcg::Mesh::VertexHandle v_handles[12];

#define CREATE_VERTEX_ON_EDGE(n, corner_i, corner_j)                                                                   \
    if(edge_table[cubeindex] & (1 << n))                                                                               \
    {                                                                                                                  \
        vertex_list[n] = interpolate_point(ISOVALUE,                                                                   \
                                           voxel_positions[corner_i],                                                  \
                                           sdf_values[corner_i],                                                       \
                                           voxel_positions[corner_j],                                                  \
                                           sdf_values[corner_j]);                                                      \
        v_handles[n]   = mesh->add_vertex(atcg::Mesh::Point(vertex_list[n].x, vertex_list[n].y, vertex_list[n].z));    \
        glm::vec3 c    = interpolate_point(ISOVALUE,                                                                   \
                                        color_values[corner_i],                                                     \
                                        sdf_values[corner_i],                                                       \
                                        color_values[corner_j],                                                     \
                                        sdf_values[corner_j]);                                                      \
        atcg::Mesh::Color clr;                                                                                         \
        clr[0] = static_cast<uint8_t>(c.x * 255.0f);                                                                   \
        clr[1] = static_cast<uint8_t>(c.y * 255.0f);                                                                   \
        clr[2] = static_cast<uint8_t>(c.z * 255.0f);                                                                   \
        mesh->set_color(v_handles[n], clr);                                                                            \
    }

            CREATE_VERTEX_ON_EDGE(0, 0, 1);
            CREATE_VERTEX_ON_EDGE(1, 1, 2);
            CREATE_VERTEX_ON_EDGE(2, 2, 3);
            CREATE_VERTEX_ON_EDGE(3, 3, 0);
            CREATE_VERTEX_ON_EDGE(4, 4, 5);
            CREATE_VERTEX_ON_EDGE(5, 5, 6);
            CREATE_VERTEX_ON_EDGE(6, 6, 7);
            CREATE_VERTEX_ON_EDGE(7, 7, 4);
            CREATE_VERTEX_ON_EDGE(8, 0, 4);
            CREATE_VERTEX_ON_EDGE(9, 1, 5);
            CREATE_VERTEX_ON_EDGE(10, 2, 6);
            CREATE_VERTEX_ON_EDGE(11, 3, 7);

#undef CREATE_VERTEX_ON_EDGE

            for(uint32_t i = 0; triangle_table[cubeindex][i] != -1; i += 3)
            {
                atcg::Mesh::VertexHandle face_vhandles[3];

                face_vhandles[0] = v_handles[triangle_table[cubeindex][i + 0]];
                face_vhandles[1] = v_handles[triangle_table[cubeindex][i + 1]];
                face_vhandles[2] = v_handles[triangle_table[cubeindex][i + 2]];
                mesh->add_face(face_vhandles[0], face_vhandles[1], face_vhandles[2]);
            }
        }
    }

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        // Create a grid at the origin with 10 voxels per side length with a size of 0.1
        grid = std::make_shared<SDFGrid>(glm::vec3(-3.5 / 2.0f), 70, 0.05f);
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

        if(render_grid)
            atcg::Renderer::drawGrid(grid->getGridDimensions(), camera_controller->getCamera(), update_grid);
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
            ImGui::MenuItem("Marching Cubes", nullptr, &show_marching_cubes);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        if(show_marching_cubes)
        {
            ImGui::Begin("Settings MC", &show_marching_cubes);
            if(ImGui::SliderFloat("Voxel Size", &voxel_size, 0.01f, 0.2f))
            {
                uint32_t num_voxels = static_cast<uint32_t>(70.0f / voxel_size * 0.05f);
                grid        = std::make_shared<SDFGrid>(glm::vec3(-static_cast<float>(num_voxels) * voxel_size / 2.0f),
                                                 num_voxels,
                                                 voxel_size);
                update_grid = true;
            }

            if(ImGui::Button("Run"))
            {
                // Fill the grid with the sdf
                mesh = std::make_shared<atcg::Mesh>();

                /// Exercise: Replace the sdf by a csg that looks similar to the supplied figure
                ///           -Hint: Surface Representations II: Slide 12

                SDFHeart sdf_heart;
                SDF* sdf = &sdf_heart;

                // 

                fillGrid(grid, sdf);

                marching_cubes(grid, mesh);

                render_grid = false;

                mesh->uploadData();
            }

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
        dispatcher.dispatch<atcg::FileDroppedEvent>(ATCG_BIND_EVENT_FN(Exercise05Layer::onFileDropped));
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
    std::shared_ptr<SDFGrid> grid;

    bool show_marching_cubes  = true;
    bool show_render_settings = false;
    bool render_faces         = true;
    bool render_points        = false;
    bool render_edges         = false;
    bool update_grid          = false;
    bool render_grid          = true;
    float voxel_size          = 0.05f;
};

class Exercise05 : public atcg::Application
{
public:
    Exercise05() : atcg::Application() { pushLayer(new Exercise05Layer("Layer")); }

    ~Exercise05() {}
};

atcg::Application* atcg::createApplication()
{
    return new Exercise05;
}