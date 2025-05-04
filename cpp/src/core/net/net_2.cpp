/*************************************************
 * @file net_2.cpp
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D network where you calculate visibility
 * @version 0.1
 * @date 2024-11-13
 * 
 * @copyright Copyright (c) 2024 Shota TABATA
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *************************************************/

// include standards
#include <string>
#include <utility>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <random>

// include header
#include "core/net/net_2.h"

// include CGAL library
#include <CGAL/intersections.h>
#include <CGAL/Bbox_2.h>
// #include <CGAL/optimal_bounding_box.h> //? 3Dしか効かない?
#include <CGAL/create_offset_polygons_2.h>
typedef CGAL::Bbox_2 Bbox_2;

// include utility
#include "core/util/std_vector_util.h"
#include "core/util/std_unordered_set_util.h"
#include "core/util/random_engine.h"

// include boost
#include <boost/shared_ptr.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

// include edge filter
#include "core/edge/edge_2_filter.h"

// include custom weight map
#include "core/weight_mapper/weight_mapper_2.h"

// include spherical geometry util
#include "core/geom/spherical_geom_util.h"

//* Constructor *//
Net_2::Net_2(const int node_num) :
    node_num(node_num)
{}

Net_2::Net_2(const int node_num, const Polygon_2 domain) :
    node_num(node_num), 
    domain(domain)
{
    // 対象領域が自己交差のない閉領域かチェックする
    if (!domain.is_simple()) {
        throw std::runtime_error(
            "The provided domain is not a simple polygon.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // 対象領域の頂点の順序を反時計回りに設定する
    if (domain.is_clockwise_oriented()) {
        this->domain.reverse_orientation();
    }

}

//** Getter **//
int Net_2::get_node_num() const {
    return node_num;
}

Polygon_2 Net_2::get_domain() const {
    return domain;
}

double Net_2::get_offset_r() const {
    return offset_r;
}

//** Setter **//
void Net_2::set_node_num(const int node_num) {
    this->node_num = node_num;
}

void Net_2::set_domain(const Polygon_2 domain) {
    // 対象領域が自己交差のない閉領域かチェックする
    if (!domain.is_simple()) {
        throw std::runtime_error(
            "The provided domain is not a simple polygon.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    this->domain = domain;
    // 対象領域の頂点の順序を反時計回りに設定する
    if (domain.is_clockwise_oriented()) {
        this->domain.reverse_orientation();
    }
}

void Net_2::set_offset_r(const double offset_r) {
    this->offset_r = offset_r;
}

//** Network Method **//
std::vector<std::shared_ptr<Node_2>> Net_2::generate_random_nodes() const {
    
    // 対象領域のバウンディングボックスを計算する
    //TODO ノード数節約のため oriented bounding box にしたい
    Bbox_2 bbox = domain.bbox();
    double xmax {bbox.xmax()};
    double xmin {bbox.xmin()};
    double ymax {bbox.ymax()};
    double ymin {bbox.ymin()};
    Point_2 p_left_bottom (xmin, ymin);
    Point_2 p_right_bottom (xmax, ymin);
    Point_2 p_right_top (xmax, ymax);
    Point_2 p_left_top (xmin, ymax);
    
    Polygon_2 bbox_polygon;
    bbox_polygon.push_back(p_left_bottom);
    bbox_polygon.push_back(p_right_bottom);
    bbox_polygon.push_back(p_right_top);
    bbox_polygon.push_back(p_left_top);

    // バウンディングボックスをオフセットする
    double bbox_area = bbox_polygon.area();
    double offset_dist = std::sqrt(bbox_area) * offset_r; //TODO オフセット長さの設定方法は暫定的
    std::vector<boost::shared_ptr<Polygon_2>> offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset_dist, bbox_polygon);

    Polygon_2 offset_bbox_polygon = *offset_polygons.at(0);
    Bbox_2 offset_bbox = offset_bbox_polygon.bbox();
    double offset_xmax {offset_bbox.xmax()};
    double offset_xmin {offset_bbox.xmin()};
    double offset_ymax {offset_bbox.ymax()};
    double offset_ymin {offset_bbox.ymin()};

    // 対象領域内にnode_num個のノードを生成する
    // 乱数生成器の設定
    std::uniform_real_distribution<> x_dist(offset_xmin, offset_xmax);
    std::uniform_real_distribution<> y_dist(offset_ymin, offset_ymax);

    std::vector<std::shared_ptr<Node_2>> node_ptrs;
    int inside_node_num {0};
    while (inside_node_num < node_num) {
        std::shared_ptr<Node_2> node_ptr = std::make_shared<Node_2>(x_dist(Random_Engine::get_engine()), y_dist(Random_Engine::get_engine()));
        node_ptrs.push_back(node_ptr);

        // 内外判定してカウントする
        switch (domain.bounded_side(*node_ptr)) {
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

void Net_2::initialize() {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

void Net_2::initialize(const std::vector<std::shared_ptr<Node_2>> node_ptrs) {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

void Net_2::remove_vertex(const Net_2::vertex_descriptor vertex) {
    boost::clear_vertex(vertex, *this);
    boost::remove_vertex(vertex, *this);
}

void Net_2::dissconnect_edges(std::vector<std::shared_ptr<Obstacle_2>> obstacle_ptrs) {
    Net_2::edge_iterator eit, eit_end;
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

std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_2::calculate_shortest_path_tree(
    const Net_2::vertex_descriptor source, 
    const size_t mode, 
    const bool using_obstacle, 
    const bool using_weight, 
    const std::vector<Net_2::vertex_descriptor> prohibited_vertices) const 
{
    
    // エッジの重みの設定
    Weight_Mapper_2 weight_mapper(*this, 
                                  mode, 
                                  using_obstacle, 
                                  using_weight, 
                                  prohibited_vertices);
    auto weight_map = boost::make_function_property_map<Net_2::edge_descriptor>(
        [&](Net_2::edge_descriptor e) { return weight_mapper(e); }
    );

    // 結果を格納する配列の初期化
    size_t num_vertices = boost::num_vertices(*this);
    std::vector<double> distances(num_vertices, std::numeric_limits<double>::infinity());
    std::vector<Net_2::vertex_descriptor> predecessors(num_vertices);

    // Dijkstra法
    boost::dijkstra_shortest_paths(*this, 
                                   source, 
                                   boost::predecessor_map(predecessors.data())
                                   .distance_map(distances.data())
                                   .weight_map(weight_map));

    // 出力    
    std::vector<std::pair<Net_2::vertex_descriptor, double>> spt;
    for (size_t i = 0; i < distances.size(); ++i) {
        spt.push_back(std::make_pair(predecessors.at(i), distances.at(i)));
    }

    return spt;

}

void Net_2::match_shortest_path_trees(
    const Net_2::vertex_descriptor root, 
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>>& spt1_r, 
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>>& spt2_r,
    std::unordered_set<Net_2::vertex_descriptor>& matching_vertices 
) const {

    std::vector<Net_2::vertex_descriptor> children1_vec = spt1_r[root];
    std::vector<Net_2::vertex_descriptor> children2_vec = spt2_r[root];

    const std::unordered_set<Net_2::vertex_descriptor> children1(children1_vec.begin(), children1_vec.end());
    const std::unordered_set<Net_2::vertex_descriptor> children2(children2_vec.begin(), children2_vec.end());

    std::unordered_set<Net_2::vertex_descriptor> sub_matching_vertices = set_intersection(children1, children2);

    matching_vertices.insert(sub_matching_vertices.begin(), sub_matching_vertices.end());
    
    if (sub_matching_vertices.size() == 0) {
        return;
    } else {
        for (const auto& child : sub_matching_vertices) {
            match_shortest_path_trees(child, spt1_r, spt2_r, matching_vertices);
        }
    }

}

void Net_2::search_visible_vertices(
    const Net_2::vertex_descriptor root, 
    const std::vector<std::pair<Net_2::vertex_descriptor, double>>& spt_without_obstacles, 
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>>& spt_without_obstacles_r, 
    std::unordered_set<Net_2::vertex_descriptor>& visible_vertices, 
    const double length_constraint) const 
{
    std::vector<Net_2::vertex_descriptor> children_vec = spt_without_obstacles_r[root];
    std::vector<Net_2::vertex_descriptor> next_parents_vec;
    std::pair<Net_2::edge_descriptor, bool> edge_existance;
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

void Net_2::search_panoramic_vision_points(
    const Net_2::vertex_descriptor root, 
    const std::vector<std::pair<Net_2::vertex_descriptor, double>>& spt_without_obstacles, 
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>>& spt_without_obstacles_r, 
    std::unordered_map<std::string, std::vector<Point_2>>& panoramic_vision_points, 
    const double length_constraint) const 
{
    std::vector<Net_2::vertex_descriptor> children_vec = spt_without_obstacles_r[root];
    std::vector<Net_2::vertex_descriptor> next_parents_vec;
    std::pair<Net_2::edge_descriptor, bool> edge_existance;
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
                panoramic_vision_points[Obstacle_2::UNKNOWN_NAME].push_back(*(*this)[child]);
            }
        } else {
            // root と child を結ぶエッジと障害物との交点を保存
            std::vector<std::pair<Point_2, std::string>> x_points;
            for (const auto& obstacle_ptr : (*this)[edge_existance.first]->get_x_obstacle_ptrs()) {
                auto intersection = CGAL::intersection(*(*this)[edge_existance.first], *obstacle_ptr);
                if (const Point_2* p = boost::get<Point_2>(&*intersection)) {
                    x_points.emplace_back(*p, obstacle_ptr->get_name());
                }
            }
            // root から見て一番近い交点を保存
            double min_distance = std::numeric_limits<double>::max();
            std::pair<Point_2, std::string> panoramic_vision_point;
            
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

void Net_2::search_reachable_vertices(
    const Net_2::vertex_descriptor root, 
    const std::vector<std::pair<Net_2::vertex_descriptor, double>>& spt_with_obstacles, 
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>>& spt_with_obstacles_r, 
    std::unordered_set<Net_2::vertex_descriptor>& reachable_vertices, 
    const double length_constraint) const 
{
    std::vector<Net_2::vertex_descriptor> children_vec = spt_with_obstacles_r[root];
    std::vector<Net_2::vertex_descriptor> next_parents_vec;
    std::pair<Net_2::edge_descriptor, bool> edge_existance;
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
std::unordered_set<Net_2::vertex_descriptor> Net_2::calculate_visible_vertices(
    Point_2 p, 
    const double length_constraint) const 
{
    //?もしかしたら障害付きと障害なしで最短経路木求めて比べたほうが早いかもしれない
    Net_2::vertex_descriptor vantage_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_2::vertex_descriptor, double>> spt_without_obstacles;

    spt_without_obstacles = this->calculate_shortest_path_tree(vantage_node, Net_2::MODE_VISIBILITY, false, true);

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> spt_without_obstacles_r;

    for (std::size_t i {0}; i < spt_without_obstacles.size(); ++i) {
        Net_2::vertex_descriptor parent = spt_without_obstacles.at(i).first;
        if (parent != i && parent != Net_2::null_vertex()) {
            spt_without_obstacles_r[parent].push_back(i);
        }
    }

    // 可視である頂点を探索する
    std::unordered_set<Net_2::vertex_descriptor> visible_vertices;

    visible_vertices.insert(vantage_node);
    search_visible_vertices(vantage_node, 
                            spt_without_obstacles, 
                            spt_without_obstacles_r,
                            visible_vertices, 
                            length_constraint);

    return visible_vertices;

}

std::unordered_set<Net_2::vertex_descriptor> Net_2::calculate_reachable_vertices(
    Point_2 p, 
    const double length_constraint) const 
{
    Net_2::vertex_descriptor orient_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_2::vertex_descriptor, double>> spt_with_obstacles;

    spt_with_obstacles = this->calculate_shortest_path_tree(orient_node, Net_2::MODE_ROUTE, true, true);
    
    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> spt_with_obstacles_r;

    for (std::size_t i {0}; i < spt_with_obstacles.size(); ++i) {
        Net_2::vertex_descriptor parent = spt_with_obstacles.at(i).first;
        if (parent != i && parent != Net_2::null_vertex()) {
            spt_with_obstacles_r[parent].push_back(i);
        }
    }

    // 到達可能である頂点を探索する
    std::unordered_set<Net_2::vertex_descriptor> reachable_vertices;

    reachable_vertices.insert(orient_node);
    search_reachable_vertices(orient_node, 
                              spt_with_obstacles, 
                              spt_with_obstacles_r,
                              reachable_vertices, 
                              length_constraint);

    return reachable_vertices;

}

std::unordered_map<std::string, std::vector<std::pair<Point_2, Point_2>>> Net_2::calculate_panoramic_vision(
    Point_2 p, 
    const double length_constraint, 
    const double radius
) const 
{
    Net_2::vertex_descriptor vantage_node = this->find_nearest_node(p);

    std::vector<std::pair<Net_2::vertex_descriptor, double>> spt_without_obstacles;

    spt_without_obstacles = this->calculate_shortest_path_tree(vantage_node, Net_2::MODE_VISIBILITY, false, true);

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> spt_without_obstacles_r;

    for (std::size_t i {0}; i < spt_without_obstacles.size(); ++i) {
        Net_2::vertex_descriptor parent = spt_without_obstacles.at(i).first;
        if (parent != i && parent != Net_2::null_vertex()) {
            spt_without_obstacles_r[parent].push_back(i);
        }
    }

    // 障害物ごとに見えている点を記録する
    std::unordered_map<std::string, std::vector<Point_2>> panoramic_vision_points;
    search_panoramic_vision_points(vantage_node, 
                                   spt_without_obstacles, 
                                   spt_without_obstacles_r, 
                                   panoramic_vision_points,
                                   length_constraint);

    // 見えている点を母点とする円周上のボロノイ図を計算する
    std::vector<Point_2> generators;
    for (const auto& elem : panoramic_vision_points) {
        generators.insert(generators.end(), elem.second.begin(), elem.second.end());
    }
    std::unordered_map<Point_2, std::pair<Point_2, Point_2>> circular_Voronoi;
    circular_Voronoi = construct_circular_Voronoi_diagram(p, generators, radius);

    std::unordered_map<std::string, std::vector<std::pair<Point_2, Point_2>>> panoramic_vision;
    for (const auto& elem : panoramic_vision_points) {
        for (const auto& panoramic_vision_point : elem.second) {
            panoramic_vision[elem.first].push_back(circular_Voronoi[panoramic_vision_point]);
        }
    }

    return panoramic_vision;

}

//** Geometric Method **//
std::vector<Point_2> Net_2::build_cell(const std::shared_ptr<Node_2> node_ptr) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

Net_2::vertex_descriptor Net_2::find_nearest_node(Point_2 p) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

//** Bool Method **//
bool Net_2::has_connection(const Graph_2::edge_descriptor& edge, const size_t mode) const {
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
void Net_2::write_nodes(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_2>> node_ptrs = collect_node_ptrs();

    double x, y;
    for (auto& node_ptr : node_ptrs) {
        x = node_ptr->x();
        y = node_ptr->y();
        f << std::scientific 
          << std::setprecision(std::numeric_limits<double>::max_digits10) 
          << x << " " << y << std::endl;
    }

}

void Net_2::write_edges(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_2>> node_ptrs = collect_node_ptrs();

    Graph_2::edge_iterator eit, eit_end;
    Graph_2::vertex_descriptor s_vertex;
    Graph_2::vertex_descriptor t_vertex;
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

void Net_2::write_adjacency(const std::string& abs_file_path, const size_t mode) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_2>> node_ptrs = collect_node_ptrs();

    Net_2::vertex_iterator vit, vit_end;
    int key_index {0};
    int value_index {0};
    Net_2::edge_descriptor edge;
    bool exist;

    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        key_index = find_index(node_ptrs, (*this)[*vit]);
        
        f << key_index;
        add_adjacent_vertices(f, node_ptrs, *vit, mode);
        f << std::endl;

        }
}

void Net_2::add_adjacent_vertices(std::ofstream& f, 
                                  const std::vector<std::shared_ptr<Node_2>>& node_ptrs, 
                                  const Graph_2::vertex_descriptor vertex, 
                                  const size_t mode) const {
    
    Graph_2::adjacency_iterator adj_vit, adj_vit_end;
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

void Net_2::write_cells(const std::string& abs_file_path) const {
    throw std::runtime_error(
        "Virtual function. Use derived class.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );
}

Polygon_2 Net_2::read_domain(const std::string& abs_file_path) {
    std::ifstream file(abs_file_path);
    Polygon_2 domain;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return domain;
    }

    std::string line;
    double x;
    double y;
    std::vector<Point_2> domain_points;
    while (std::getline(file, line)) {
        std::istringstream line_stream(line);
        line_stream >> x >> y;

        domain_points.emplace_back(x, y);

    }
    
    domain = Polygon_2(domain_points.begin(), domain_points.end());
    return domain;
    
}

//** Utility Method **/
std::vector<std::shared_ptr<Node_2>> Net_2::collect_node_ptrs() const {
    std::vector<Graph_2::vertex_descriptor> vertices {}; // すべての頂点
    Graph_2::vertex_iterator vit, vit_end; // 頂点のイテレータ
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        vertices.push_back(*vit);
    }

    auto vertices_num = boost::num_vertices(*this);
    int vid {0};
    std::vector<std::shared_ptr<Node_2>> node_ptrs {vertices_num};
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        vid = find_index(vertices, *vit);
        node_ptrs.at(vid) = (*this)[*vit];
    }

    return node_ptrs;
}