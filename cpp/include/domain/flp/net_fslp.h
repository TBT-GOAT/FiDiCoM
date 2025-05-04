/*************************************************
 * @file net_fslp.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D netowrk for facility sign location problem
 * @version 0.1
 * @date 2025-04-19
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

#ifndef NET_FSLP_H
#define NET_FSLP_H

// include STL
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>

// include Net_2
#include "core/net/net_2.h"

class Net_FSLP {
    private:       
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_FACILITIES = -1;       // 未初期化判定用サービス供給点割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_FACILITIES;                                                // ダミーサービス供給点の点
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_SIGNS = -2;            // 未初期化判定用サイン割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_SIGNS;                                                     // ダミーサインの点
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS = -3;   // 未初期化判定用主要建物割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_MAIN_BUILDINGS;                                            // ダミー主要建物の点

        Point_2 inner_point;                                // 需要のある空間の内部点
        double visible_length = Net_FSLP::VISIBLE_LENGTH;   // 可視長さ //TODO サービス供給点とサインの可視長さを分ける
        
        std::vector<Net_2::vertex_descriptor> demands {};          // 需要点
        std::vector<Net_2::vertex_descriptor> facilities {};       // サービス供給点
        std::vector<Net_2::vertex_descriptor> signs {};            // サイン（視認できたとき最寄りのサービス供給点を指示する）
        std::vector<Net_2::vertex_descriptor> main_buildings {};   // 主要建物（到着したとき最寄りのサービス供給点を指示する）
        
        Net_2::vertex_descriptor dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;           // サービス供給点割当用ダミー頂点
        Net_2::vertex_descriptor dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;                     // サイン割当用ダミー頂点
        Net_2::vertex_descriptor dummy_vertex_main_buildings = UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS;   // 主要建物割当用ダミー頂点
        std::vector<std::pair<Net_2::vertex_descriptor, double>> facility_coverage_tree {};                 // 可視であるサービス供給点を示す最短経路木 
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > facility_coverage_trees {};                                                                  // 可視であるサービス供給点を示す最短経路木
        std::vector<std::pair<Net_2::vertex_descriptor, double>> facility_shortest_path_tree {};       // 最寄りのサービス供給点を示す最短経路木
        std::vector<std::pair<Net_2::vertex_descriptor, double>> sign_coverage_tree {};                // 可視であるサインを示す最短経路木
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > sign_coverage_trees {};                                                                      // 可視であるサインを示す最短経路木
        std::vector<std::pair<Net_2::vertex_descriptor, double>> main_building_shortest_path_tree {};  // 最寄りの主要建物割当用最短経路木
        
        // 割当＝可視かつ最寄り
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_demand;       // 需要点に対するサービス供給点の割当（キー：需要点，値：サービス供給点）
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> sign_assignment_to_demand;           // 需要点に対するサインの割当（キー：需要点，値：サイン）
        // 割当＝最寄り
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_sign;         // サインに対するサービス供給点の割当（キー：サイン，値：サービス供給点）
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_main_building;// 主要建物に対するサービス供給点の割当（キー：主要建物，値：サービス供給点）
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> main_building_assignment_to_demand;  // 需要点に対する主要建物の割当（キー：需要点，値：主要建物）

        //** Network Methods **//
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_facilities();
        /*************************************************
         * @brief サインを束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_signs();
        /*************************************************
         * @brief 主要建物を束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_main_buildings();
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_facilities();
        /*************************************************
         * @brief サインを束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_signs();
        /*************************************************
         * @brief 主要建物を束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_main_buildings();
        /*************************************************
         * @brief サービス供給点から可視である範囲を計算するための最短経路木を構築する
         * !バグ．最も近いサービス供給点から見えなかったら見えないことになる．
         * 
         *************************************************/
        void build_facility_coverage_tree();
        /*************************************************
         * @brief サービス供給点から可視である範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_coverage_trees();
        /*************************************************
         * @brief サービス供給点から最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_shortest_path_tree();
        /*************************************************
         * @brief サインから可視である範囲を計算するための最短経路木を構築する
         * !バグ．最も近いサインから見えなかったら見えないことになる．
         * 
         *************************************************/
        void build_sign_coverage_tree();
        /*************************************************
         * @brief サインから可視である範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_sign_coverage_trees();
        /*************************************************
         * @brief 主要建物から最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_main_building_shortest_path_tree();
        /*************************************************
         * @brief 最寄りの主要建物までの経路を計算する
         * 
         * @param demand 
         * @return std::deque<Net_2::vertex_descriptor> 
         *************************************************/
        std::deque<Net_2::vertex_descriptor> calculate_path_to_main_building(Net_2::vertex_descriptor demand) const;

        //** Initialization Methods **//
        /*************************************************
         * @brief 指定した点に最も近い需要点を探索する
         * 
         * @param points 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> search_nearest_demands(const std::vector<Point_2> points) const;
        /*************************************************
         * @brief 需要点をランダムに選択する
         * 
         * @param selected_num 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> select_demands_at_random(const size_t selected_num);
        
        //** Constraint Method **//
        /*************************************************
         * @brief 需要点に可視である最寄りのサービス供給点を割り当てる
         * 
         *************************************************/
        void assign_facility_to_demand();
        /*************************************************
         * @brief 需要点に可視である最寄りのサインを割り当てる
         * 
         *************************************************/
        void assign_sign_to_demand();
        /*************************************************
         * @brief サインに最寄りのサービス供給点を割り当てる
         * 
         *************************************************/
        void assign_facility_to_sign();
        /*************************************************
         * @brief 主要建物に最寄りのサービス供給点を割り当てる
         * 
         *************************************************/
        void assign_facility_to_main_building();
        /*************************************************
         * @brief 需要点に最寄りの主要建物を割り当てる
         * 
         *************************************************/
        void assign_main_building_to_demand();
        /*************************************************
         * @brief 需要点に対するサービス供給点の割当を更新する
         * 
         *************************************************/
        void update_facility_assignment_to_demand();

    public:
        static constexpr double VISIBLE_LENGTH = 100000.0; // 可視の長さのデフォルト値（100m）
        
        std::shared_ptr<Net_2> net_ptr; // 任意のNet_2のポインタ

        //** Constructor **//
        Net_FSLP();
        Net_FSLP(std::shared_ptr<Net_2> net_ptr);

        //** Getter **//
        Point_2 get_inner_point() const;
        double get_visible_length() const;

        std::vector<Net_2::vertex_descriptor> get_demands() const;
        std::vector<Net_2::vertex_descriptor> get_facilities() const;
        std::vector<Net_2::vertex_descriptor> get_signs() const;
        std::vector<Net_2::vertex_descriptor> get_main_buildings() const;

        Net_2::vertex_descriptor get_dummy_vertex_facilities() const;
        Net_2::vertex_descriptor get_dummy_vertex_signs() const;
        Net_2::vertex_descriptor get_dummy_vertex_main_buildings() const;

        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_facility_coverage_tree() const;
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > get_facility_coverage_trees() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_facility_shortest_path_tree() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_sign_coverage_tree() const;
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > get_sign_coverage_trees() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_main_building_shortest_path_tree() const;

        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_facility_assignment_to_demand() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_sign_assignment_to_demand() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_facility_assignment_to_sign() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_facility_assignment_to_main_building() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_main_building_assignment_to_demand() const;

        //** Setter **//
        void set_inner_point(const Point_2 inner_point);
        void set_visible_length(const double visible_length);

        void set_demands();
        void set_demands(const Point_2 inner_point);
        void set_demands(const std::vector<Net_2::vertex_descriptor> demands);
        void set_facilities(const std::vector<Net_2::vertex_descriptor> facilities);
        void set_signs(const std::vector<Net_2::vertex_descriptor> signs);
        void set_main_buildings(const std::vector<Net_2::vertex_descriptor> main_buildings);

        void set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities);
        void set_dummy_vertex_signs(const Net_2::vertex_descriptor dummy_vertex_signs);
        void set_dummy_vertex_main_buildings(const Net_2::vertex_descriptor dummy_vertex_main_buildings);

        //** Network Methods **//
        /*************************************************
         * @brief すべてのダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertices();
        /*************************************************
         * @brief ダミー頂点が束ねるサービス供給点を更新する
         * 
         *************************************************/
        void update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                            const Net_2::vertex_descriptor next_facility_vertex);
        /*************************************************
         * @brief ダミー頂点が束ねるサインを更新する
         * 
         * @param prev_sign_vertex 
         * @param next_sign_vertex 
         *************************************************/
        void update_dummy_vertex_signs(const Net_2::vertex_descriptor prev_sign_vertex, 
                                       const Net_2::vertex_descriptor next_sign_vertex);
        /*************************************************
         * @brief 各種の割当用の最短経路木を構築する
         * 
         *************************************************/
        void build_trees();
        /*************************************************
         * @brief 各種の割当用の最短経路木を削除する
         * 
         *************************************************/
        void clear_trees();

        //** Initialization Methods **//
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param demand_points 
         *************************************************/
        void initialize_facilities(const std::vector<Point_2> facility_points);
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param facility_num 
         *************************************************/
        void initialize_facilities(const size_t facility_num);
        /*************************************************
         * @brief サインを初期化する
         * 
         * @param sign_points 
         *************************************************/
        void initialize_signs(const std::vector<Point_2> sign_points);
        /*************************************************
         * @brief サインを初期化する
         * 
         * @param sign_num 
         *************************************************/
        void initialize_signs(const size_t sign_num);
        /*************************************************
         * @brief 主要建物を初期化する
         * 
         * @param main_building_points 
         *************************************************/
        void initialize_main_buildings(const std::vector<Point_2> main_building_points);

        /*************************************************
         * @brief 最寄りの需要点を探索する
         * 
         * @param point 
         * @return Net_2::vertex_descriptor 
         *************************************************/
        Net_2::vertex_descriptor search_nearest_demand(const Point_2 point) const;

        //** Finalization Methods **//
        /*************************************************
         * @brief 設定をクリアする（すべてのダミー頂点を削除する）
         * 
         *************************************************/
        void clear();

        //** Constraint Method **//
        /*************************************************
         * @brief 需要点に対するサービス供給点の割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_facility_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 需要点に対するサインの割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_sign_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief サインに対するサービス供給点の割当を取得する
         * 
         * @param sign 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_facility_to_sign(Net_2::vertex_descriptor sign) const;
        /*************************************************
         * @brief 主要建物に対するサービス供給点の割当を取得する
         * 
         * @param main_building 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_facility_to_main_building(Net_2::vertex_descriptor main_building) const;
        /*************************************************
         * @brief 需要点に対する主要建物の割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_main_building_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 各種の割当を実行する
         * 
         *************************************************/
        void build_assignments();
        /*************************************************
         * @brief 割当をクリアする
         * 
         *************************************************/
        void clear_assignments();

        //** Cost Function Methods **//
        /*************************************************
         * @brief 需要点のコストを計算する
         * TODO 面積の考慮
         * 
         * @param demand 
         * @return double 
         *************************************************/
        double calculate_cost(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief サービス供給点が割当済のときのコストを計算する
         * 
         * @param demand 
         * @return double 
         *************************************************/
        double calculate_cost_to_nearest_facility_identified(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief サービス供給点が割当前のときのコストを計算する
         * 
         * @param demand 
         * @return double 
         *************************************************/
        double calculate_cost_to_nearest_facility_unidentified(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 中継地点（サービス供給点が判明する頂点）を探索する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> find_waystop(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 中継地点を経由したときのコストを計算する
         * 
         * @param demand 
         * @param waystop 
         * @return double 
         *************************************************/
        double calculate_cost_via_waystop(Net_2::vertex_descriptor demand, Net_2::vertex_descriptor waystop) const;
        /*************************************************
         * @brief 主要建物を経由したときのコストを計算する
         * 
         * @param demand 
         * @param assigned_main_building 
         * @return double 
         *************************************************/
        double calculate_cost_via_main_building(Net_2::vertex_descriptor demand, Net_2::vertex_descriptor assigned_main_building) const;

};

#endif // NET_FSLP_H