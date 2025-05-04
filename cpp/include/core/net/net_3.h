/*************************************************
 * @file net_3.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 3D network
 * @version 0.1
 * @date 2025-01-07
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

#ifndef NET_3_H
#define NET_3_H

// include standards
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <memory>

// include Node_3, Edge_3, Obstacle_3
#include "core/node/node_3.h"
#include "core/edge/edge_3.h"
#include "core/obstacle/obstacle_3.h"

// include boost graph library
#include <boost/graph/adjacency_list.hpp>
typedef boost::adjacency_list<
    boost::vecS, 
    boost::vecS, 
    boost::undirectedS, 
    std::shared_ptr<Node_3>, 
    std::shared_ptr<Edge_3>
> Graph_3;

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Projection_on_sphere_traits_3.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef CGAL::Triangle_3<K> Triangle_3;
typedef CGAL::Projection_on_sphere_traits_3<K> Traits;
typedef Traits::Point_3                        Point_on_sphere_3;

class Net_3 : public Graph_3 {
    protected:       
        int          node_num;               // 対象領域内のノード数
        Polyhedron_3 domain;                 // 対象領域
        double       offset_r {OFFSET_R};    // 対象領域をオフセットするときの係数
    
        //** Network Method **//
        /*************************************************
         * @brief ネットワークのノードを生成する
         * 
         * @return std::vector<std::shared_ptr<Node_3>> 
         *************************************************/
        std::vector<std::shared_ptr<Node_3>> generate_random_nodes() const;
        /*************************************************
         * @brief 最短経路木の一致部分を探索する
         * 
         * @param root 
         * @param spt1 
         * @param spt2 
         * @param matching_vertices 
         *************************************************/
        void match_shortest_path_trees(
            const Net_3::vertex_descriptor root, 
            std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt1_r, 
            std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt2_r,
            std::unordered_set<Net_3::vertex_descriptor>& matching_vertices 
        ) const;

        //** Planning Method **//
        /*************************************************
         * @brief 再帰的に視覚を構成する点（障害物と交差する点）を探索する
         * 
         * @param root 
         * @param spt_without_obstacles 
         * @param spt_without_obstacles_r 
         * @param panoramic_vision_points key: 障害物の名前, value: 点
         * @param length_constraint
         *************************************************/
        void search_panoramic_vision_points(
            const Net_3::vertex_descriptor root, 
            const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_without_obstacles, 
            std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_without_obstacles_r, 
            std::unordered_map<std::string, std::vector<Point_3>>& panoramic_vision_points, 
            const double length_constraint
        ) const;

        //** file IO method **//
        /*************************************************
         * @brief 隣接する頂点をファイルに書き込む
         * 
         * @param f 
         * @param node_ptrs 
         * @param vertex 
         * @param mode 
         *************************************************/
        void add_adjacent_vertices(std::ofstream& f, 
                                   const std::vector<std::shared_ptr<Node_3>>& node_ptrs, 
                                   const Graph_3::vertex_descriptor vertex, 
                                   const size_t mode) const;

    public:
        static constexpr double OFFSET_R = 0.05;        // 対象領域をオフセットするときの係数のデフォルト値
        static constexpr size_t SEED = 17;              // 乱数生成器のシード値
        static constexpr size_t MODE_VISIBILITY = 0;    // 可視性計算モード
        static constexpr size_t MODE_ROUTE = 1;         // 経路計算モード
        
        //** Constructor **//
        using Graph_3::Graph_3;
        /*************************************************
         * @brief Construct a new Net_3 object
         * 
         * @param node_num 
         *************************************************/
        Net_3(const int node_num);
        /*************************************************
         * @brief Construct a new Net_3 object
         * 
         * @param node_num 
         * @param domain 
         *************************************************/
        Net_3(const int node_num, const Polyhedron_3 domain);
        
        //** Getter **//
        /*************************************************
         * @brief 対象領域内のノード数を取得する
         * 
         * @return int 
         *************************************************/
        int get_node_num() const;
        /*************************************************
         * @brief 対象領域を取得する
         * 
         * @return Polyhedron_3 
         *************************************************/
        Polyhedron_3 get_domain() const;
        /*************************************************
         * @brief 対象領域をオフセットするときの係数を取得する
         * 
         * @return double 
         *************************************************/
        double get_offset_r() const;

        //** Setter **//
        /*************************************************
         * @brief 対象領域内のノード数を設定する
         * 
         * @param node_num 
         *************************************************/
        void set_node_num(const int node_num);
        /*************************************************
         * @brief 対象領域を設定する
         * 
         * @param domain 
         *************************************************/
        void set_domain(const Polyhedron_3 domain);
        /*************************************************
         * @brief 対象領域をオフセットするときの係数を取得する
         * 
         * @param offset_r 
         *************************************************/
        void set_offset_r(const double offset_r);
        
        //** Network Method **//
        /*************************************************
         * @brief ネットワークを初期化する
         * 
         *************************************************/
        virtual void initialize();
        /*************************************************
         * @brief 与えられた頂点でネットワークを初期化する
         * 
         * @param node_ptrs 
         *************************************************/
        virtual void initialize(const std::vector<std::shared_ptr<Node_3>> node_ptrs);
        /*************************************************
         * @brief 頂点を削除する
         * 
         * @param vertex 
         *************************************************/
        void remove_vertex(const Net_3::vertex_descriptor vertex);
        /*************************************************
         * @brief 障害物と交差するエッジを切断する
         * 
         * @param obstacle_ptrs 
         *************************************************/
        void dissconnect_edges(std::vector<std::shared_ptr<Obstacle_3>> obstacle_ptrs);
        /*************************************************
         * @brief 最短経路木を計算する
         * 
         * @param source 
         * @param mode 
         * @param using_obstacle 
         * @param using_weight 
         * @param prohibited_vertices
         * @return std::vector<std::pair<Net3::vertex_descriptor, double>> 
         *************************************************/
        std::vector<std::pair<Net_3::vertex_descriptor, double>> calculate_shortest_path_tree(
            const Net_3::vertex_descriptor source, 
            const size_t mode, 
            const bool using_obstacle=true, 
            const bool using_weight=true, 
            const std::vector<Net_3::vertex_descriptor> prohibited_vertices={}
        ) const;
        /*************************************************
         * @brief 再帰的に可視の頂点を探索する
         * 
         * @param root 
         * @param spt_without_obstacles 
         * @param spt_without_obstacle_r 
         * @param visible_vertices 
         * @param length_constraint 
         *************************************************/
        void search_visible_vertices(
            const Net_3::vertex_descriptor root, 
            const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_without_obstacles, 
            std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_without_obstacles_r, 
            std::unordered_set<Net_3::vertex_descriptor>& visible_vertices, 
            const double length_constraint=std::numeric_limits<double>::max()
        ) const;
        /*************************************************
         * @brief 再帰的に到達可能な頂点を探索する
         * 
         * @param root 
         * @param spt_with_obstacles 
         * @param spt_with_obstacles_r 
         * @param reachable_vertices 
         * @param length_constraint 
         *************************************************/
        void search_reachable_vertices(
            const Net_3::vertex_descriptor root, 
            const std::vector<std::pair<Net_3::vertex_descriptor, double>>& spt_with_obstacles, 
            std::unordered_map<Net_3::vertex_descriptor, std::vector<Net_3::vertex_descriptor>>& spt_with_obstacles_r, 
            std::unordered_set<Net_3::vertex_descriptor>& reachable_vertices, 
            const double length_constraint=std::numeric_limits<double>::max()
        ) const;

        //** Geometric Method **//
        /*************************************************
         * @brief 頂点に紐づくセルを構成する
         * 
         * @param node_ptr 
         * @return Polyhedron_3 
         *************************************************/
        virtual Polyhedron_3 build_cell(const std::shared_ptr<Node_3> node_ptr) const;
        /*************************************************
         * @brief 与えられた点に最も近い頂点を探索する
         * 
         * @param p 
         * @return Net_3::vertex_descriptor 
         *************************************************/
        virtual Net_3::vertex_descriptor find_nearest_node(Point_3 p) const;

        //** Planning Method **//
        /*************************************************
         * @brief 可視の頂点を計算する
         * 
         * @param p 
         * @param length_constraint //! 最短経路木に沿った重み付き長さ（ユークリッド距離ではない）で制約をかける
         * @return std::unordered_set<Net_3::vertex_descriptor> 
         *************************************************/
        std::unordered_set<Net_3::vertex_descriptor> calculate_visible_vertices(
            Point_3 p, 
            const double length_constraint=std::numeric_limits<double>::max()
        ) const;
        /*************************************************
         * @brief 到達可能な頂点を計算する
         * 
         * @param p 
         * @param length_constraint //! 最短経路木に沿った重み付き長さ（ユークリッド距離ではない）で制約をかける
         * @return std::unordered_set<Net_3::vertex_descriptor> 
         *************************************************/
        std::unordered_set<Net_3::vertex_descriptor> calculate_reachable_vertices(
            Point_3 p, 
            const double length_constraint=std::numeric_limits<double>::max()
        ) const;
        /*************************************************
         * @brief 全天球視野を計算する
         * 
         * @param p 
         * @param length_constraint 
         * @return std::unordered_map<std::string, std::vector<std::vector<Point_on_sphere_3>>> 
         *************************************************/
        std::unordered_map<std::string, std::vector<std::vector<Point_on_sphere_3>>> calculate_panoramic_vision(
            Point_3 p, 
            const double length_constraint=std::numeric_limits<double>::max(), 
            const double radius = 1.0
        ) const;

        //** Bool Method **//
        bool has_connection(const Graph_3::edge_descriptor& edge, const size_t mode) const;
        
        //** File IO Method **//
        /*************************************************
         * @brief 頂点の座標をファイルに書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        void write_nodes(const std::string& abs_file_path) const;
        /*************************************************
         * @brief 辺をファイルに書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        void write_edges(const std::string& abs_file_path) const;
        /*************************************************
         * @brief 隣接配列をファイルに書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        void write_adjacency(const std::string& abs_file_path, const size_t mode) const;
        /*************************************************
         * @brief 頂点に紐づくセルを書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        virtual void write_cells(const std::string& abs_file_path) const;
        /*************************************************
         * @brief 頂点に紐づくセルを書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        virtual void write_cells(const std::string& abs_file_path, const std::vector<std::shared_ptr<Node_3>> node_ptrs) const;
        /*************************************************
         * @brief 対象領域を読み込む
         * 
         * @param abs_file_path 
         * @return Polyhedron_3
         *************************************************/
        static Polyhedron_3 read_domain(const std::string& abs_file_path);

        //** Utility Method **//
        /*************************************************
         * @brief 頂点を格納するベクトルを作成する
         * 
         * @return std::vector<std::shared_ptr<Node_3>> 
         *************************************************/
        std::vector<std::shared_ptr<Node_3>> collect_node_ptrs() const;

};

#endif // NET_3_H