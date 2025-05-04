
// include header
#include "core/obstacle/obstacle_3.h"

// include Edge_3
#include "core/edge/edge_3.h"

// include standard
#include <filesystem>

// include CGAL library
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <CGAL/Polygon_mesh_processing/connected_components.h>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/IO/STL.h>
#include <CGAL/boost/graph/IO/STL.h>

typedef K::Point_3 Point_3_; // Triangulate_3のprivateメンバ変数と区別するため

// include standard util
#include "core/util/std_string_util.h"

//** Constructor **//
Obstacle_3::Obstacle_3(const Triangle_3 triangle_3) : 
    Triangle_3(triangle_3)
{}

Obstacle_3::Obstacle_3(const Triangle_3 triangle_3, 
                       const bool is_transparent, 
                       const bool is_navigable, 
                       const bool is_domain, 
                       const std::string name) :
    Triangle_3(triangle_3), 
    is_transparent(is_transparent),
    is_navigable(is_navigable), 
    is_domain(is_domain), 
    name(name)
{}

Obstacle_3::Obstacle_3(const Triangle_3 triangle_3, 
                       const bool is_transparent, 
                       const bool is_navigable, 
                       std::unordered_set<std::shared_ptr<Edge_3>>&& x_edge_ptrs, 
                       const bool is_domain, 
                       const std::string name) :
    Triangle_3(triangle_3), 
    is_transparent(is_transparent),
    is_navigable(is_navigable),
    x_edge_ptrs(std::move(x_edge_ptrs)), 
    is_domain(is_domain), 
    name(name)
{}

//** Getter **//
bool Obstacle_3::get_is_transparent() const {
    return is_transparent;
}

bool Obstacle_3::get_is_navigable() const {
    return is_navigable;
}

std::unordered_set<std::shared_ptr<Edge_3>> Obstacle_3::get_x_edge_ptrs() const {
    return x_edge_ptrs;
}

std::string Obstacle_3::get_name() const {
    return name;
}

//** Setter **//
void Obstacle_3::set_is_transparent(bool is_transparent) {
    this->is_transparent = is_transparent;
}

void Obstacle_3::set_is_navigable(bool is_navigable) {
    this->is_navigable = is_navigable;
}

void Obstacle_3::set_x_edge_ptrs(std::unordered_set<std::shared_ptr<Edge_3>>&& x_edge_ptrs) {
    this->x_edge_ptrs = std::move(x_edge_ptrs);
}

void Obstacle_3::set_name(std::string name) {
    this->name = name;
}

//** Construction Method **//
std::vector<std::shared_ptr<Obstacle_3>> Obstacle_3::convert_polyhedron(Polyhedron_3& polyhedron,  
                                                                        const bool is_transparent, 
                                                                        const bool is_navigable, 
                                                                        const bool is_domain, 
                                                                        const std::string name) {
    
    // Surface_meshに変換
    Surface_mesh surface_mesh;
    CGAL::copy_face_graph(polyhedron, surface_mesh);


    // 障害物に変換
    std::vector<std::shared_ptr<Obstacle_3>> obstacle_ptrs;
    obstacle_ptrs = convert_surface_mesh(surface_mesh, 
                                         is_transparent, 
                                         is_navigable, 
                                         is_domain, 
                                         name);

    return obstacle_ptrs;
}

std::vector<std::shared_ptr<Obstacle_3>> Obstacle_3::convert_surface_mesh(Surface_mesh& surface_mesh,  
                                                                          const bool is_transparent, 
                                                                          const bool is_navigable, 
                                                                          const bool is_domain, 
                                                                          const std::string name) {
    
    std::vector<std::shared_ptr<Obstacle_3>> obstacle_ptrs;

    // 多面体を三角形に分割
    CGAL::Polygon_mesh_processing::triangulate_faces(surface_mesh);

    for (const auto& face : faces(surface_mesh)) {
        // 面の頂点を取得
        std::vector<Point_3_> points;
        for (const auto& vertex : vertices_around_face(surface_mesh.halfedge(face), surface_mesh)) {
            points.push_back(surface_mesh.point(vertex));
        }

        // 障害物の登録
        if (points.size() == 3) { 
            // 面が三角形の場合
            Triangle_3 triangle(points[0], 
                                points[1],
                                points[2]);
            obstacle_ptrs.push_back(std::make_shared<Obstacle_3>(triangle, 
                                                                 is_transparent, 
                                                                 is_navigable, 
                                                                 is_domain, 
                                                                 name));
        } else {
            std::cerr << "Error: non-triangular face left in polyhedron." << std::endl;
        }

    }

    return obstacle_ptrs;
}

void Obstacle_3::insert_x_edge_ptr(std::shared_ptr<Edge_3> edge_ptr) {
    this->x_edge_ptrs.insert(edge_ptr);
}

void Obstacle_3::erase_x_edge_ptr(std::shared_ptr<Edge_3> edge_ptr) {
    this->x_edge_ptrs.erase(edge_ptr);
}

//** Geometric Method **//
bool Obstacle_3::is_intersecting(const Segment_3 segment_3) const {
    return CGAL::do_intersect(*this, segment_3);
}

//** File IO Method **//
std::vector<std::shared_ptr<Obstacle_3>> Obstacle_3::read_obstacles(const std::string& abs_file_path, 
                                                                    const bool is_transparent, 
                                                                    const bool is_navigable, 
                                                                    const bool is_domain) {
    std::ifstream file(abs_file_path);
    std::string obstacle_name = std::filesystem::path(abs_file_path).stem().string(); // ファイル名 = 障害物の名前
    Surface_mesh sm;
    std::vector<std::shared_ptr<Obstacle_3>> obstacle_ptrs;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return obstacle_ptrs;
    }

    std::string file_extension = get_file_extension(abs_file_path);
    if (file_extension == "obj") {
        if (!CGAL::IO::read_OBJ(file, sm)) {
            std::cerr << "Error: Failed to read OBJ file." << std::endl;
        }
    } else if (file_extension == "stl") {
        if (!CGAL::IO::read_STL(file, sm)) {
            std::cerr << "Error: Failed to read STL file." << std::endl;
        }
    } else {
        file >> sm;
    }
    
    obstacle_ptrs = convert_surface_mesh(sm, is_transparent, is_navigable, is_domain, obstacle_name);
    
    /*//** 重み付き領域を考える場合の参考として残しておく 
    // 空間的に離れている箇所で分割
    // 連結部分のマップの作成
    typedef boost::graph_traits<Surface_mesh>::face_descriptor face_descriptor;
    Surface_mesh::Property_map<face_descriptor, size_t> fccmap = sm.add_property_map<face_descriptor, size_t>("f:CC").first;

    // 連結部分を識別
    size_t num = CGAL::Polygon_mesh_processing::connected_components(sm, fccmap);
    std::cout << "The mesh contains " << num << " connected components." << std::endl;

    // 連結部分のインデックスごとに面を登録
    typedef std::map<std::size_t, std::vector<face_descriptor>> ComponentsMap;
    ComponentsMap components;

    for (face_descriptor f : faces(sm)) {
        components[fccmap[f]].push_back(f);
    } 

    // 連結部分を復元
    for (const auto& component : components) {
        Surface_mesh component_mesh;
        std::map<Surface_mesh::Vertex_index, Surface_mesh::Vertex_index> vertex_map;
        
        for (face_descriptor f : component.second) {
            // 連結成分を構成する面を追加
            std::vector<Surface_mesh::Vertex_index> vertices;
            for (auto v : vertices_around_face(sm.halfedge(f), sm)) {
                if (vertex_map.find(v) == vertex_map.end()) {
                    // 頂点 v が未登録
                    vertex_map[v] = component_mesh.add_vertex(sm.point(v));
                }
                vertices.push_back(vertex_map[v]);
            }
            component_mesh.add_face(vertices);
        }

        // 必要に応じて処理

    }
    */

    return obstacle_ptrs;

}