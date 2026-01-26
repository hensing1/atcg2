#include "OpenMesh/Core/Mesh/Handles.hh"
#include "OpenMesh/Core/Mesh/TriConnectivity.hh"
#include <cmath>
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

class Exercise11Layer : public atcg::Layer
{
public:
    Exercise11Layer(const std::string& name) : atcg::Layer(name) {}

    enum class WeightType
    {
        UNIFORM_SPRING,
        CHORDAL_SPRING,
        WACHSPRESS,
        DISCRETE_HARMONIC,
        MEAN_VALUE
    };

    std::vector<OpenMesh::SmartEdgeHandle> detect_boundary_edges(const std::shared_ptr<atcg::Mesh>& mesh)
    {
        std::vector<OpenMesh::SmartEdgeHandle> boundary_edges;

        /// Exercise: - Detect boundary edges
        ///           - You can use OpenMesh functions for this

        OpenMesh::SmartHalfedgeHandle firstBoundaryEdge;
        for (auto he_it = mesh->halfedges_begin(); he_it != mesh->halfedges_end(); he_it++) {
            if (mesh->is_boundary(*he_it)) {
                firstBoundaryEdge = *he_it;
                boundary_edges.push_back(he_it->edge());
                break;
            }
        }
        OpenMesh::SmartHalfedgeHandle nextEdge = firstBoundaryEdge.next();
        while (nextEdge.idx() != firstBoundaryEdge.idx()) {
            if (!nextEdge.is_boundary()) {
                std::cout << "ah shit fuck" << std::endl;
                auto a = (int *)0;
                *a = 4;
            }
            boundary_edges.push_back(nextEdge.edge());
            nextEdge = nextEdge.next();
        }

        return boundary_edges;
    }

    std::vector<OpenMesh::SmartVertexHandle> detect_boundary_path(const std::shared_ptr<atcg::Mesh>& mesh,
                                                   const std::vector<OpenMesh::SmartEdgeHandle>& boundary_edges)
    {
        std::vector<OpenMesh::SmartVertexHandle> boundary_path;

        // VertexHandle start      = mesh->from_vertex_handle(mesh->halfedge_handle(boundary_edges[0], 0));
        // VertexHandle current_to = mesh->to_vertex_handle(mesh->halfedge_handle(boundary_edges[0], 0));
        // boundary_path.push_back(start);
        // boundary_path.push_back(current_to);

        /// Exercise: -Find the path of boundary edges
        ///           -Hint: This is an older exercise which I ported into this framework. There are more elegant
        ///           solutions but it is ok if your code is O(n^2) where n is the number of boundary edges

        // whoops we already solved this one in a)

        for (auto edge : boundary_edges) {
            auto he = edge.halfedge(0);
            auto outer = he.is_boundary() ? he : he.opp();
            boundary_path.push_back(outer.from());
        }

        return boundary_path;
    }

    std::vector<double> path_length(const std::shared_ptr<atcg::Mesh>& mesh, const std::vector<OpenMesh::SmartVertexHandle>& path)
    {
        std::vector<double> path_lengths;

        /// Exercise: Compute the edge lengths

        for (size_t i = 0; i < path.size() - 1; i++) {
            auto diff = mesh->point(path[i]) - mesh->point(path[i+1]);
            path_lengths.push_back(diff.length());
        }
        path_lengths.push_back((mesh->point(path[path.size()-1]) - mesh->point(path[0])).length());

        return path_lengths;
    }

    std::vector<atcg::Mesh::Point> map_boundary_edges_to_circle(const std::vector<double>& edge_lengths)
    {
        double total_length = std::accumulate(edge_lengths.begin(), edge_lengths.end(), 0.0);

        std::vector<atcg::Mesh::Point> circle;
        // circle.push_back({1.0f, 0.0f, 0.0f});

        /// Exercise: -Map boundary edges onto a circle
        ///           -Begin with an angle of zero and increment it relative to the current edge length
        ///           -Use the parameterization of a circle in 2D. The third coordinate can be z = 0

        size_t num_edges = edge_lengths.size();
        double cumul_length = 0;
        const double TAU = 6.2831853071;
        for (size_t i = 0; i < num_edges; i++) {
            double angle = TAU * cumul_length / total_length;  // small-angle approximation I hope this is ok
            circle.push_back({cos(angle), sin(angle), 0});
            cumul_length += edge_lengths[i];
        }

        return circle;
    }

    inline double angle_from_metric(double a, double b, double c)
    {
        /* numerically stable version of law of cosines
         * angle between a and b, opposite to edge c
         */

        double alpha = acos((a * a + b * b - c * c) / (2.0 * a * b));

        if(alpha < 1e-8f)
        {
            alpha = std::sqrt((c * c - (a - b) * (a - b)) / (2.0 * a * b));
            std::cout << "small angle < 1e-8!" << std::endl;
        }
        return alpha;
    }

    std::vector<Eigen::Triplet<double>> method_switcher(const std::shared_ptr<atcg::Mesh>& mesh,
                                                        const WeightType& method)
    {
        std::vector<Eigen::Triplet<double>> coefficients;

        size_t n_faces = mesh->n_faces();

        for(auto f_it = mesh->faces_begin(); f_it != mesh->faces_end(); ++f_it)
        {
            std::vector<VertexHandle> v;
            for(auto v_it = f_it->vertices().begin(); v_it != f_it->vertices().end(); ++v_it) { v.push_back(*v_it); }
            assert(v.size() == 3);

            atcg::Mesh::Point vi = mesh->point(v[0]);
            atcg::Mesh::Point vj = mesh->point(v[1]);
            atcg::Mesh::Point vk = mesh->point(v[2]);

            int i = v[0].idx();
            int j = v[1].idx();
            int k = v[2].idx();

            double rij = (vj - vi).norm();
            double rjk = (vk - vj).norm();
            double rki = (vi - vk).norm();

            double alphai = angle_from_metric(rij, rki, rjk);
            double alphaj = angle_from_metric(rjk, rij, rki);
            double alphak = angle_from_metric(rjk, rki, rij);

            double wij = 0, wjk = 0, wki = 0;
            double wji = 0, wkj = 0, wik = 0;

            /// Exercise: -Calculate the weights
            ///           -Keep the relationship between i and j in mind.
            ///           -Remember that Eigen::SparseMatrix.setFromTriplets adds values with the same indices

            // why would we iterate over faces instead of half edges :(
            // iterating over half edges would guarantee that every i-j pair would only appear once
            // now we have to constantly worry about wether or not we are looking at a boundary edge
            switch(method)
            {
                case WeightType::UNIFORM_SPRING:
                {
                    // implement here uniform weights
                    wij = 0.5, wjk = 0.5, wki = 0.5;
                    wji = 0.5, wkj = 0.5, wik = 0.5;
                }
                break;

                case WeightType::CHORDAL_SPRING:
                {
                    // implement here chordal spring weitghts: w = 1.0 / r^2
                    wij = 0.5 / (rij * rij), wjk = 0.5 / (rjk * rjk), wki = 0.5 / (rki * rki); 
                    wji = 0.5 / (rij * rij), wkj = 0.5 / (rjk * rjk), wik = 0.5 / (rki * rki);
                }
                break;

                case WeightType::WACHSPRESS:
                {
                    // implement here the wachspress weights

                }
                break;

                case WeightType::DISCRETE_HARMONIC:
                {
                    // implement here the discrete harmonic weights
                    //
                }
                break;

                case WeightType::MEAN_VALUE:
                {
                    // implement here the mean value weights
                    wij = tan(alphai / 2) / rij, wjk = tan(alphaj / 2) / rjk, wki = tan(alphak / 2) / wki;
                    wji = tan(alphaj / 2) / rij, wkj = tan(alphak / 2) / rjk, wik = tan(alphai / 2) / wki;
                }
                break;
            }

            coefficients.emplace_back(i, j, wij);
            coefficients.emplace_back(j, k, wjk);
            coefficients.emplace_back(k, i, wki);

            // symmetric part
            coefficients.emplace_back(j, i, wji);
            coefficients.emplace_back(k, j, wkj);
            coefficients.emplace_back(i, k, wik);
        }

        return coefficients;
    }

    Eigen::SparseMatrix<double> construct_operator(const std::shared_ptr<atcg::Mesh>& mesh,
                                                   const std::vector<Eigen::Triplet<double>>& coefficients,
                                                   const std::vector<OpenMesh::SmartVertexHandle>& path)
    {
        Eigen::SparseMatrix<double> op(mesh->n_vertices(), mesh->n_vertices());
        /// Exercise: -Compute the operator (Slide 8)
        ///           -The given coefficients can be created to build a sparse matrix W = w_ij
        ///           -w_ii = 0
        ///           -The operator has to be normalized such that the row sums = 0 and L_ii = 1
        ///           -Instead of separating boundary and interior vertices into the lhs and rhs of the linear system of
        ///           equations, we describe the whole system in one equation Ax=b,
        ///            for this replace rows corresponding to boundary conditions to delta rows
        ///           -You can get row sums by computing op * 1, where 1 is a vector of ones

        op.setFromTriplets(coefficients.begin(), coefficients.end());
        Eigen::VectorXd ones(mesh->n_vertices());
        ones.setOnes();
        auto sums = op * ones;

        Eigen::VectorXd normalizer = sums.cwiseInverse();
        for (auto boundary : path) {
            normalizer(boundary.idx()) = 0;
        }
        op = normalizer.asDiagonal() * op;  // boundary rows are now 0, interior rows are normed to 1

        op *= -1;  // negative weights
        op += ones.asDiagonal();  // 1's on the diagonal

        return op;
    }

    Eigen::MatrixXd construct_rhs(const std::shared_ptr<atcg::Mesh>& mesh,
                                  const std::vector<OpenMesh::SmartVertexHandle>& path,
                                  const std::vector<atcg::Mesh::Point>& boundary_constraints)
    {
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(mesh->n_vertices(), 3);

        for(uint32_t i = 0; i < boundary_constraints.size(); ++i)
        {
            rhs(path[i].idx(), 0) = boundary_constraints[i][0];
            rhs(path[i].idx(), 1) = boundary_constraints[i][1];
            rhs(path[i].idx(), 2) = boundary_constraints[i][2];
        }

        return rhs;
    }

    Eigen::MatrixXd solve(const Eigen::SparseMatrix<double>& op, const Eigen::MatrixXd& rhs)
    {
        Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
        solver.compute(op);
        return solver.solve(rhs).eval();
    }

    Eigen::MatrixXd reparameterize(const std::shared_ptr<atcg::Mesh>& mesh,
                                   const std::vector<OpenMesh::SmartVertexHandle>& path,
                                   const std::vector<atcg::Mesh::Point>& constraints,
                                   const WeightType& method)
    {
        std::vector<Eigen::Triplet<double>> coefficients = method_switcher(mesh, method);

        Eigen::SparseMatrix<double> op = construct_operator(mesh, coefficients, path);

        Eigen::MatrixXd rhs = construct_rhs(mesh, path, constraints);

        Eigen::MatrixXd uv = solve(op, rhs);

        return uv;
    }

    void apply_parameterization(const std::shared_ptr<atcg::Mesh>& mesh, const Eigen::MatrixXd& uv)
    {
        for(auto v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            atcg::Mesh::Point p {uv(v_it->idx(), 0), uv(v_it->idx(), 1), 0.0f};
            mesh->set_point(*v_it, p);
        }
    }

    // This is run at the start of the program
    virtual void onAttach() override
    {
        const auto& window = atcg::Application::get()->getWindow();
        float aspect_ratio = (float)window->getWidth() / (float)window->getHeight();
        camera_controller  = std::make_shared<atcg::CameraController>(aspect_ratio);

        mesh_original = atcg::IO::read_mesh("res/maxear.obj");
        // mesh_original->request_vertex_colors();

        mesh = std::make_shared<atcg::Mesh>(*mesh_original.get());

        atcg::normalize(mesh);

        boundary_edges = detect_boundary_edges(mesh_original);
        boundary_path  = detect_boundary_path(mesh_original, boundary_edges);
        edge_lengths   = path_length(mesh_original, boundary_path);
        circle         = map_boundary_edges_to_circle(edge_lengths);

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

        if(ImGui::BeginMenu("Exercise"))
        {
            ImGui::MenuItem("Show Reparameterization Settings", nullptr, &show_rep_settings);
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

        if(show_rep_settings)
        {
            ImGui::Begin("Reparameterization Settings", &show_rep_settings);

            ImGui::Text("Weighting scheme:");
            if(ImGui::Button("Uniform Spring"))
            {
                auto uv = reparameterize(mesh_original, boundary_path, circle, WeightType::UNIFORM_SPRING);
                apply_parameterization(mesh, uv);
                mesh->uploadData();
            }

            if(ImGui::Button("Chordal Spring"))
            {
                auto uv = reparameterize(mesh_original, boundary_path, circle, WeightType::CHORDAL_SPRING);
                apply_parameterization(mesh, uv);
                mesh->uploadData();
            }

            if(ImGui::Button("Wachspress"))
            {
                auto uv = reparameterize(mesh_original, boundary_path, circle, WeightType::WACHSPRESS);
                apply_parameterization(mesh, uv);
                mesh->uploadData();
            }

            if(ImGui::Button("Discrete Harmonic"))
            {
                auto uv = reparameterize(mesh_original, boundary_path, circle, WeightType::DISCRETE_HARMONIC);
                apply_parameterization(mesh, uv);
                mesh->uploadData();
            }

            if(ImGui::Button("Mean Value"))
            {
                auto uv = reparameterize(mesh_original, boundary_path, circle, WeightType::MEAN_VALUE);
                apply_parameterization(mesh, uv);
                mesh->uploadData();
            }

            ImGui::End();
        }
    }

    // This function is evaluated if an event (key, mouse, resize events, etc.) are triggered
    virtual void onEvent(atcg::Event& event) override
    {
        camera_controller->onEvent(event);

        atcg::EventDispatcher dispatcher(event);
    }

private:
    std::shared_ptr<atcg::CameraController> camera_controller;
    std::shared_ptr<atcg::Mesh> mesh;
    std::shared_ptr<atcg::Mesh> mesh_original;

    std::vector<OpenMesh::SmartEdgeHandle> boundary_edges;
    std::vector<OpenMesh::SmartVertexHandle> boundary_path;
    std::vector<double> edge_lengths;
    std::vector<atcg::Mesh::Point> circle;

    bool show_render_settings = false;
    bool show_rep_settings    = true;
    bool render_faces         = false;
    bool render_points        = false;
    bool render_edges         = true;
};

class Exercise11 : public atcg::Application
{
public:
    Exercise11() : atcg::Application() { pushLayer(new Exercise11Layer("Layer")); }

    ~Exercise11() {}
};

atcg::Application* atcg::createApplication()
{
    return new Exercise11;
}
