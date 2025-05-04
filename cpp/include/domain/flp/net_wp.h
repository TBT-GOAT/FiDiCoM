/*************************************************
 * @file net_wp.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D network for Weber problem
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

#ifndef NET_WP_H
#define NET_WP_H

// include STL
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>

// include Net_2
#include "core/net/net_2.h"

class Net_WP {
    private:    
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX = -1;  // 未初期化判定用サービス供給点割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT;                                           // ダミーサービス供給点の点

        Point_2 inner_point; // 需要のある空間の内部点
        
        std::vector<Net_2::vertex_descriptor> demands {};          // 需要点
        std::vector<Net_2::vertex_descriptor> facilities {};       // サービス供給点
        
        Net_2::vertex_descriptor dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX;              // サービス供給点割当用ダミー頂点
        std::vector<std::pair<Net_2::vertex_descriptor, double>> facility_shortest_path_tree {};    // 最寄りのサービス供給点を示す最短経路木
        
        //** Network Methods **//
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_facilities();
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_facilities();
        /*************************************************
         * @brief サービス供給点から最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_shortest_path_tree();

        //** Initialization Methods **//
        /*************************************************
         * @brief 指定した点に最も近い需要点を探索する
         * 
         * @param points 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> search_nearest_demands(const std::vector<Point_2> points);
        /*************************************************
         * @brief 需要点をランダムに選択する
         * 
         * @param selected_num 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> select_demands_at_random(const size_t selected_num);
        
    public:
        
        std::shared_ptr<Net_2> net_ptr; // 任意のNet_2のポインタ

        //** Constructor **//
        Net_WP();
        Net_WP(std::shared_ptr<Net_2> net_ptr);
        
        //** Getter **//
        Point_2 get_inner_point() const;

        std::vector<Net_2::vertex_descriptor> get_demands() const;
        std::vector<Net_2::vertex_descriptor> get_facilities() const;

        Net_2::vertex_descriptor get_dummy_vertex_facilities() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_facility_shortest_path_tree() const;

        //** Setter **//
        void set_inner_point(const Point_2 inner_point);

        void set_demands();
        void set_demands(const Point_2 inner_point);
        void set_demands(const std::vector<Net_2::vertex_descriptor> demands);
        void set_facilities(const std::vector<Net_2::vertex_descriptor> facilities);

        void set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities);

        //** Network Methods **//
        /*************************************************
         * @brief ダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex();
        /*************************************************
         * @brief ダミー頂点が束ねるサービス供給点を更新する
         * 
         * @param prev_facility_vertex 
         * @param next_facility_vertex 
         *************************************************/
        void update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                            const Net_2::vertex_descriptor next_facility_vertex);
        /*************************************************
         * @brief サービス供給点割当用の最短経路木を構築する
         * 
         *************************************************/
        void build_tree();
        /*************************************************
         * @brief サービス供給点割当用の最短経路木を削除する
         * 
         *************************************************/
        void clear_tree();

        //** Initialization Methods **//
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param facility_points 
         *************************************************/
        void initialize_facilities(const std::vector<Point_2> facility_points);
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param facility_num 
         *************************************************/
        void initialize_facilities(const size_t facility_num);

        //** Finalization Methods **//
        /*************************************************
         * @brief 設定をクリアする（ダミー頂点を削除する）
         * 
         *************************************************/
        void clear();

        //** Cost Function Methods **//
        /*************************************************
         * @brief 需要点のコストを計算する
         * 
         * @param demand 
         * @return double 
         *************************************************/
        double calculate_cost(Net_2::vertex_descriptor demand) const;

};

#endif // NET_WP_H