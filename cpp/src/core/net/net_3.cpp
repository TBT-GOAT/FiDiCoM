

// include standards
#include <string>
#include <utility>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <random>

// include header
#include "core/net/net_3.h"

// include CGAL library
#include <CGAL/intersections.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/optimal_bounding_box.h> //? 3Dしか効かない?
#include <CGAL/Side_of_triangle_mesh.h>

#include <CGAL/IO/OBJ.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/IO/STL.h>

typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

// include utility
#include "core/util/std_vector_util.h"
#include "core/util/std_unordered_set_util.h"
#include "core/util/std_string_util.h"
#include "core/util/random_engine.h"

// include boost
#include <boost/shared_ptr.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/function_property_map.hpp>

// include edge filter
#include "core/edge/edge_3_filter.h"

// include custom weight map
#include "core/weight_mapper/weight_mapper_3.h"

// include spherical geometry util
#include "core/geom/spherical_geom_util.h"

//* Constructor *//
Net_3::Net_3(const int node_num) :
    node_num(node_num)
{}

Net_3::Net_3(const int node_num, const Polyhedron_3 domain) :
    node_num(node_num), 
    domain(domain)
{    
    // 対象領域が閉じているかチェックする
    if (!domain.is_closed()) {
        throw std::runtime_error(
            "The provided domain is not a closed polyhedron.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
    // 対象領域に自己交差がないかチェックする
    std::vector<std::pair<boost::graph_traits<Polyhedron_3>::face_descriptor, 
                          boost::graph_traits<Polyhedron_3>::face_descriptor>> intersected_faces;
    CGAL::Polygon_mesh_processing::self_intersections(
        faces(domain),
        domain,
        std::back_inserter(intersected_faces)
    );

    if (!intersected_faces.empty()) {
        // throw std::runtime_error(
        //     "The provided domain is not a simple polyhedron.\n"
        //     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        // );
        std::cerr 
            << "The provided domain is not a simple polyhedron.\n"
            << "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        << std::endl;
    }

}

//** Getter **//
int Net_3::get_node_num() const {
    return node_num;
}

Polyhedron_3 Net_3::get_domain() const {
    return domain;
}

double Net_3::get_offset_r() const {
    return offset_r;
}

//** Setter **//
void Net_3::set_node_num(const int node_num) {
    this->node_num = node_num;
}

void Net_3::set_domain(const Polyhedron_3 domain) {
    // 対象領域が閉じているかチェックする
    if (!domain.is_closed()) {
        throw std::runtime_error(
            "The provided domain is not a closed polyhedron.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // 対象領域に自己交差がないかチェックする
    std::vector<std::pair<boost::graph_traits<Polyhedron_3>::face_descriptor, 
                          boost::graph_traits<Polyhedron_3>::face_descriptor>> intersected_faces;
    CGAL::Polygon_mesh_processing::self_intersections(
        faces(domain),
        domain,
        std::back_inserter(intersected_faces)
    );

    if (!intersected_faces.empty()) {
        throw std::runtime_error(
            "The provided domain is not a simple polyhedron.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    this->domain = domain;

}

void Net_3::set_offset_r(const double offset_r) {
    this->offset_r = offset_r;
}

//** Network Method **//
std::vector<std::shared_ptr<Node_3>> Net_3::generate_random_nodes() const {
    
    // 対象領域のバウンディングボックスを計算する
    //TODO ノード数節約のため oriented bounding box にしたい
    std::array<Point_3, 8> obb_points;
    CGAL::oriented_bounding_box(domain, obb_points);

    // バウンディングボックスをメッシュに変換
    Surface_mesh obb_sm;
    CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                          obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);

    // ある頂点で交わる3つの辺の方向ベクトルでバウンディングボックスを表現
    Surface_mesh::vertex_index origin_vid = *(obb_sm.vertices_begin());
    std::vector<K::Vector_3> base_vectors;

    for (const auto& halfedge : halfedges_around_target(obb_sm.halfedge(origin_vid), obb_sm)) {
        auto target = obb_sm.point(obb_sm.source(halfedge));
        auto source = obb_sm.point(origin_vid);

        base_vectors.push_back(target - source);
    }

    // バウンディングボックスをオフセットする
    K::Vector_3 origin_vector(Point_3(CGAL::ORIGIN), obb_sm.point(origin_vid));
    double bbox_volume = CGAL::Polygon_mesh_processing::volume(obb_sm);
    double offset_dist = std::cbrt(bbox_volume) * offset_r; //TODO オフセット長さの設定方法は暫定的

    // 媒介変数表示
    double len_base_vector0 = std::sqrt(base_vectors.at(0).squared_length());
    double len_base_vector1 = std::sqrt(base_vectors.at(1).squared_length());
    double len_base_vector2 = std::sqrt(base_vectors.at(2).squared_length());
    double offset_t0 = offset_dist / len_base_vector0;
    double t0_min = 0 - offset_t0;
    double t0_max = 1 + offset_t0;
    double offset_t1 = offset_dist / len_base_vector1;
    double t1_min = 0 - offset_t1;
    double t1_max = 1 + offset_t1;
    double offset_t2 = offset_dist / len_base_vector2;
    double t2_min = 0 - offset_t2;
    double t2_max = 1 + offset_t2;

    // 対象領域内にnode_num個のノードを生成する
    // 乱数生成器の設定
    std::uniform_real_distribution<> t0_dist(t0_min, t0_max);
    std::uniform_real_distribution<> t1_dist(t1_min, t1_max);
    std::uniform_real_distribution<> t2_dist(t2_min, t2_max);

    std::vector<std::shared_ptr<Node_3>> node_ptrs;

    int inside_node_num {0};
    CGAL::Side_of_triangle_mesh<Polyhedron_3, K> inside(domain); // 内外判定器

    while (inside_node_num < node_num) {

        K::Vector_3 position_vector = origin_vector 
                                      + t0_dist(Random_Engine::get_engine()) * base_vectors.at(0) 
                                      + t1_dist(Random_Engine::get_engine()) * base_vectors.at(1) 
                                      + t2_dist(Random_Engine::get_engine()) * base_vectors.at(2);

        std::shared_ptr<Node_3> node_ptr = std::make_shared<Node_3>(position_vector.x(), 
                                                                    position_vector.y(), 
                                                                    position_vector.z());
        node_ptrs.push_back(node_ptr);

        // 内外判定してカウントする
        switch (inside(*node_ptr)) {
            case CGAL::ON_BOUNDED_SIDE:
                ++inside_node_num;
                break;
            case CGAL::ON_BOUNDARY:
                break;
            case CGAL::ON_UNBOUNDED_SIDE:
                break;
        }
    }

    return node_ptrs;

}

void Net_3::initialize() {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

void Net_3::initialize(const std::vector<std::shared_ptr<Node_3>> node_ptrs) {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

void Net_3::remove_vertex(const Net_3::vertex_descriptor vertex) {
    boost::clear_vertex(vertex, *this);
    boost::remove_vertex(vertex, *this);
}

void Net_3::dissconnect_edges(std::vector<std::shared_ptr<Obstacle_3>> obstacle_ptrs) {
    Net_3::edge_iterator eit, eit_end;
    //TODO エッジの数 × 障害物の数 だけ計算している．OpenCLを使って省力化したい 
    for (boost::tie(eit, eit_end) = boost::edges(*this); eit != eit_end; ++eit) {
        for (auto obstacle_ptr : obstacle_ptrs) {
            if (obstacle_ptr->is_intersecting(*(*this)[*eit])) {
                (*this)[*eit]->set_is_visible((*this)[*eit]->get_is_visible() && obstacle_ptr->get_is_transparent());
                (*this)[*eit]->set_is_passable((*this)[*eit]->get_is_passable() && obstacle_ptr->get_is_navigable());

                obstacle_ptr->insert_x_edge_ptr((*this)[*eit]);
                (*this)[*eit]->insert_x_obstacle_ptr(obstacle_ptr);
            }
        }
    }
}

std::vector<std::pair<Net_3::vertex_descriptor, double>> Net_3::calculate_shortest_path_tree(
    const Net_3::vertex_descriptor source, 
    const size_t mode, 
    const bool using_obstacle, 
    const bool using_weight, 
    const std::vector<Net_3::vertex_descriptor> prohibited_vertices) const 
{
    
    // エッジの重みの設定
    Weight_Mapper_3 weight_mapper(*this, 
                                  mode, 
                                  using_obstacle, 
                                  using_weight, 
                                  prohibited_vertices);
    auto weight_map = boost::make_function_property_map<Net_3::edge_descriptor>(
        [&](Net_3::edge_descriptor e) { return weight_mapper(e); }
    );

    // 結果を格納する配列の初期化
    size_t num_vertices = boost::num_vertices(*this);
    std::vector<double> distances(num_vertices, std::numeric_limits<double>::infinity());
    std::vector<Net_3::vertex_descriptor> predecessors(num_vertices);

    // Dijkstra法
    boost::dijkstra_shortest_paths(*this, 
                                   source, 
                                   boost::predecessor_map(predecessors.data())
                                   .distance_map(distances.data())
                                   .weight_map(weight_map));

    // 出力    
    std::vector<std::pair<Net_3::vertex_descriptor, double>> spt;
    for (size_t i = 0; i < distances.size(); ++i) {
        spt.push_back(std::make_pair(predecessors.at(i), distances.at(i)));
    }

    return spt;

}

void Net_3::match_shortest_path_trees(
    const Net_3::vertex_descriptor root, 
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt1_r, 
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt2_r,
    std::unordered_set<Net_3::vertex_descriptor>& matching_vertices 
) const {

    std::vector<Net_3::vertex_descriptor> children1_vec = spt1_r[root];
    std::vector<Net_3::vertex_descriptor> children2_vec = spt2_r[root];

    const std::unordered_set<Net_3::vertex_descriptor> children1(children1_vec.begin(), children1_vec.end());
    const std::unordered_set<Net_3::vertex_descriptor> children2(children2_vec.begin(), children2_vec.end());

    std::unordered_set<Net_3::vertex_descriptor> sub_matching_vertices = set_intersection(children1, children2);

    matching_vertices.insert(sub_matching_vertices.begin(), sub_matching_vertices.end());
    
    if (sub_matching_vertices.size() == 0) {
        return;
    } else {
        for (const auto& child : sub_matching_vertices) {
            match_shortest_path_trees(child, spt1_r, spt2_r, matching_vertices);
        }
    }

}

void Net_3::search_visible_vertices(
    const Net_3::vertex_descriptor root, 
    const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_without_obstacles, 
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_without_obstacles_r, 
    std::unordered_set<Net_3::vertex_descriptor>& visible_vertices, 
    const double length_constraint) const 
{
    std::vector<Net_3::vertex_descriptor> children_vec = spt_without_obstacles_r[root];
    std::vector<Net_3::vertex_descriptor> next_parents_vec;
    std::pair<Net_3::edge_descriptor, bool> edge_existance;
    for (const auto& child : children_vec) {
        edge_existance = boost::edge(root, child, *this);

        if (!edge_existance.second) {
            throw std::runtime_error("Invalid adjacency (" + std::to_string(root) + " and " + std::to_string(child) + ").\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        if ((*this)[edge_existance.first]->get_is_visible()) {
            // 見通せる
            if (spt_without_obstacles.at(child).second <= length_constraint) {
                // 認識できる距離にある
                visible_vertices.insert(child);
                next_parents_vec.push_back(child);
            }
        }
    }

    if (next_parents_vec.size() == 0) {
        return;
    } else {
        for (const auto& parent : next_parents_vec) {
            search_visible_vertices(parent, 
                                    spt_without_obstacles, 
                                    spt_without_obstacles_r, 
                                    visible_vertices, 
                                    length_constraint);
        }
    }

    return;

}

void Net_3::search_panoramic_vision_points(
    const Net_3::vertex_descriptor root, 
    const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_without_obstacles, 
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_without_obstacles_r, 
    std::unordered_map<std::string, std::vector<Point_3>>& panoramic_vision_points, 
    const double length_constraint) const 
{
    std::vector<Net_3::vertex_descriptor> children_vec = spt_without_obstacles_r[root];
    std::vector<Net_3::vertex_descriptor> next_parents_vec;
    std::pair<Net_3::edge_descriptor, bool> edge_existance;
    for (const auto& child : children_vec) {
        edge_existance = boost::edge(root, child, *this);

        if (!edge_existance.second) {
            throw std::runtime_error("Invalid adjacency (" + std::to_string(root) + " and " + std::to_string(child) + ").\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        if ((*this)[edge_existance.first]->get_is_visible()) {
            // 見通せる
            if (spt_without_obstacles.at(child).second <= length_constraint) {
                // 認識できる距離にある
                next_parents_vec.push_back(child);
            } else {
                panoramic_vision_points[Obstacle_3::UNKNOWN_NAME].push_back(*(*this)[child]);
            }
        } else {
            // root と child を結ぶエッジと障害物との交点（複数）を保存
            std::vector<std::pair<Point_3, std::string>> x_points;
            for (const auto& obstacle_ptr : (*this)[edge_existance.first]->get_x_obstacle_ptrs()) {
                auto intersection = CGAL::intersection(*(*this)[edge_existance.first], *obstacle_ptr);
                if (const Point_3* p = boost::get<Point_3>(&*intersection)) {
                    x_points.emplace_back(*p, obstacle_ptr->get_name());
                }
            }
            // root から見て一番近い交点を保存
            double min_distance = std::numeric_limits<double>::max();
            std::pair<Point_3, std::string> panoramic_vision_point;
            
            for (const auto& x_point : x_points) {
                double distance = CGAL::squared_distance(*(*this)[root], x_point.first);
                if (distance < min_distance) {
                    min_distance = distance;
                    panoramic_vision_point = x_point;
                }
            }

            panoramic_vision_points[panoramic_vision_point.second].push_back(panoramic_vision_point.first);

        }
    }

    if (next_parents_vec.size() == 0) {
        return;
    } else {
        for (const auto& parent : next_parents_vec) {
            search_panoramic_vision_points(parent, 
                                 spt_without_obstacles, 
                                 spt_without_obstacles_r, 
                                 panoramic_vision_points, 
                                 length_constraint);
        }
    }

    return;
}

void Net_3::search_reachable_vertices(
    const Net_3::vertex_descriptor root, 
    const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_with_obstacles, 
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_with_obstacles_r, 
    std::unordered_set<Net_3::vertex_descriptor>& reachable_vertices, 
    const double length_constraint) const 
{
    std::vector<Net_3::vertex_descriptor> children_vec = spt_with_obstacles_r[root];
    std::vector<Net_3::vertex_descriptor> next_parents_vec;
    std::pair<Net_3::edge_descriptor, bool> edge_existance;
    for (const auto& child : children_vec) {
        edge_existance = boost::edge(root, child, *this);

        if (!edge_existance.second) {
            throw std::runtime_error("Invalid adjacency (" + std::to_string(root) + " and " + std::to_string(child) + ").\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        if ((*this)[edge_existance.first]->get_is_passable()) {
            // 通過できる
            if (spt_with_obstacles.at(child).second <= length_constraint) {
                // 到達できる距離にある
                reachable_vertices.insert(child);
                next_parents_vec.push_back(child);
            }
        }
        
    }

    if (next_parents_vec.size() == 0) {
        return;
    } else {
        for (const auto& parent : next_parents_vec) {
            search_reachable_vertices(parent, 
                                      spt_with_obstacles, 
                                      spt_with_obstacles_r, 
                                      reachable_vertices, 
                                      length_constraint);
        }
    }

    return;

}

//** Planning Method **//
std::unordered_set<Net_3::vertex_descriptor> Net_3::calculate_visible_vertices(
    Point_3 p, 
    const double length_constraint) const 
{
    Net_3::vertex_descriptor vantage_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_3::vertex_descriptor, double>> spt_without_obstacles;

    spt_without_obstacles = this->calculate_shortest_path_tree(vantage_node, Net_3::MODE_VISIBILITY, false, true);

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>> spt_without_obstacles_r;

    for (std::size_t i {0}; i < spt_without_obstacles.size(); ++i) {
        Net_3::vertex_descriptor parent = spt_without_obstacles.at(i).first;
        if (parent != i && parent != Net_3::null_vertex()) {
            spt_without_obstacles_r[parent].push_back(i);
        }
    }

    // 可視である頂点を探索する
    std::unordered_set<Net_3::vertex_descriptor> visible_vertices;

    visible_vertices.insert(vantage_node);
    search_visible_vertices(vantage_node, 
                            spt_without_obstacles, 
                            spt_without_obstacles_r,
                            visible_vertices, 
                            length_constraint);

    return visible_vertices;

}

std::unordered_set<Net_3::vertex_descriptor> Net_3::calculate_reachable_vertices(
    Point_3 p, 
    const double length_constraint) const 
{
    Net_3::vertex_descriptor orient_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_3::vertex_descriptor, double>> spt_with_obstacles;

    spt_with_obstacles = this->calculate_shortest_path_tree(orient_node, Net_3::MODE_ROUTE, true, true);
    
    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>> spt_with_obstacles_r;

    for (std::size_t i {0}; i < spt_with_obstacles.size(); ++i) {
        Net_3::vertex_descriptor parent = spt_with_obstacles.at(i).first;
        if (parent != i && parent != Net_3::null_vertex()) {
            spt_with_obstacles_r[parent].push_back(i);
        }
    }

    // 到達可能である頂点を探索する
    std::unordered_set<Net_3::vertex_descriptor> reachable_vertices;

    reachable_vertices.insert(orient_node);
    search_reachable_vertices(orient_node, 
                              spt_with_obstacles, 
                              spt_with_obstacles_r,
                              reachable_vertices, 
                              length_constraint);

    return reachable_vertices;

}

std::unordered_map<std::string, std::vector<std::vector<Point_on_sphere_3>>> Net_3::calculate_panoramic_vision(
    Point_3 p, 
    const double length_constraint, 
    const double radius
) const {
    Net_3::vertex_descriptor vantage_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_3::vertex_descriptor, double>> spt_without_obstacles;

    spt_without_obstacles = this->calculate_shortest_path_tree(vantage_node, Net_3::MODE_VISIBILITY, false, true);

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>> spt_without_obstacles_r;

    for (std::size_t i {0}; i < spt_without_obstacles.size(); ++i) {
        Net_3::vertex_descriptor parent = spt_without_obstacles.at(i).first;
        if (parent != i && parent != Net_3::null_vertex()) {
            spt_without_obstacles_r[parent].push_back(i);
        }
    }

    // 障害物ごとに見えている点を記録する
    std::unordered_map<std::string, std::vector<Point_3>> panoramic_vision_points;
    search_panoramic_vision_points(vantage_node, 
                                   spt_without_obstacles, 
                                   spt_without_obstacles_r, 
                                   panoramic_vision_points,
                                   length_constraint);

    // 見えている点を母点とする球面ボロノイ図を計算する
    std::vector<Point_3> generators;
    for (const auto& elem : panoramic_vision_points) {
        generators.insert(generators.end(), elem.second.begin(), elem.second.end());
    }
    std::unordered_map<Point_3, std::vector<Point_on_sphere_3>> spherical_Voronoi;
    spherical_Voronoi = construct_spherical_Voronoi_diagram(p, generators, radius);

    std::unordered_map<std::string, std::vector<std::vector<Point_on_sphere_3>>> panoramic_vision;
    for (const auto& elem : panoramic_vision_points) {
        for (const auto& panoramic_vision_point : elem.second) {
            panoramic_vision[elem.first].push_back(spherical_Voronoi[panoramic_vision_point]);
        }
    }

    return panoramic_vision;

}

//** Geometric Method **//
Polyhedron_3 Net_3::build_cell(const std::shared_ptr<Node_3> node_ptr) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

Net_3::vertex_descriptor Net_3::find_nearest_node(Point_3 p) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

//** Bool Method **//
bool Net_3::has_connection(const Graph_3::edge_descriptor& edge, const size_t mode) const {
    switch (mode) {
        case MODE_VISIBILITY:
            return (*this)[edge]->get_is_visible();
        case MODE_ROUTE:
            return (*this)[edge]->get_is_passable();
        default:
            throw std::runtime_error("Invalid adjacency calculation mode (" + std::to_string(mode) + ").\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

//** File IO Method **/
void Net_3::write_nodes(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_3>> node_ptrs = collect_node_ptrs();

    double x, y, z;
    for (auto& node_ptr : node_ptrs) {
        x = node_ptr->x();
        y = node_ptr->y();
        z = node_ptr->z();
        f << std::scientific 
          << std::setprecision(std::numeric_limits<double>::max_digits10) 
          << x << " " << y << " " << z << std::endl;
    }

}

void Net_3::write_edges(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_3>> node_ptrs = collect_node_ptrs();

    Graph_3::edge_iterator eit, eit_end;
    Graph_3::vertex_descriptor s_vertex;
    Graph_3::vertex_descriptor t_vertex;
    int sid;
    int tid;
    for (boost::tie(eit, eit_end) = boost::edges(*this); eit != eit_end; ++eit) {
        s_vertex = boost::source(*eit, *this);
        t_vertex = boost::target(*eit, *this);
        
        sid = find_index(node_ptrs, (*this)[s_vertex]);
        tid = find_index(node_ptrs, (*this)[t_vertex]);
        f << sid << " " << tid << " " << (*this)[*eit]->get_is_visible() << " " << (*this)[*eit]->get_is_passable() << std::endl;
    }
}

void Net_3::write_adjacency(const std::string& abs_file_path, const size_t mode) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_3>> node_ptrs = collect_node_ptrs();

    Net_3::vertex_iterator vit, vit_end;
    int key_index {0};
    int value_index {0};
    Net_3::edge_descriptor edge;
    bool exist;

    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        key_index = find_index(node_ptrs, (*this)[*vit]);
        
        f << key_index;
        add_adjacent_vertices(f, node_ptrs, *vit, mode);
        f << std::endl;

        }
}

void Net_3::add_adjacent_vertices(std::ofstream& f, 
                                  const std::vector<std::shared_ptr<Node_3>>& node_ptrs, 
                                  const Graph_3::vertex_descriptor vertex, 
                                  const size_t mode) const {
    
    Graph_3::adjacency_iterator adj_vit, adj_vit_end;
    for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(vertex, *this); adj_vit != adj_vit_end; ++adj_vit) {
        auto [edge, exist] = boost::edge(vertex, *adj_vit, *this);
        
        if (!exist) {
            std::runtime_error("Invalid adjacency.\n"
                                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        if (has_connection(edge, mode)) {
            int value_index = find_index(node_ptrs, (*this)[*adj_vit]);
            f << " " << value_index;
        }

    }
}

void Net_3::write_cells(const std::string& abs_file_path) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

void Net_3::write_cells(const std::string& abs_file_path, const std::vector<std::shared_ptr<Node_3>> node_ptrs) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

Polyhedron_3 Net_3::read_domain(const std::string& abs_file_path) {
    std::ifstream file(abs_file_path);
    Polyhedron_3 domain;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return domain;
    }

    std::string file_extension = get_file_extension(abs_file_path);
    if (file_extension == "obj") {
        if (!CGAL::IO::read_OBJ(file, domain)) {
            std::cerr << "Error: Failed to read OBJ file." << std::endl;
        }
    } else if (file_extension == "stl") {
        if (!CGAL::IO::read_STL(file, domain)) {
            std::cerr << "Error: Failed to read STL file." << std::endl;
        }
    } else {
        file >> domain;
    }

    return domain;
    
}



//** Utility Method **/
std::vector<std::shared_ptr<Node_3>> Net_3::collect_node_ptrs() const {
    std::vector<Graph_3::vertex_descriptor> vertices {}; // すべての頂点
    Graph_3::vertex_iterator vit, vit_end; // 頂点のイテレータ
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        vertices.push_back(*vit);
    }

    auto vertices_num = boost::num_vertices(*this);
    int vid {0};
    std::vector<std::shared_ptr<Node_3>> node_ptrs {vertices_num};
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        vid = find_index(vertices, *vit);
        node_ptrs.at(vid) = (*this)[*vit];
    }

    return node_ptrs;
}