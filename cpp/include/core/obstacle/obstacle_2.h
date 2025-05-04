/*************************************************
 * @file obstacle_2.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D obstacle
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

#ifndef OBSTACLE_2_H
#define OBSTACLE_2_H

// include standards
#include <iostream>
#include <string>
#include <vector>
#include <memory>

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Polygon_2<K> Polygon_2;

// Edge_2を前方宣言
class Edge_2;

class Obstacle_2 : public Segment_2 {
    private:
        bool is_transparent = false;                        // 見通せる障害物かどうか
        bool is_navigable = false;                          // 通れる障害物かどうか //? 見通せないのに通れる障害物？カーテン？
        bool is_domain = false;                             // 対象領域かどうか
        std::unordered_set<std::shared_ptr<Edge_2>> x_edge_ptrs;   // 障害物と交差しているエッジ
        std::string name;                                   // 名前
    public:
        static constexpr const char* DOMAIN_NAME = "domain";    // 対象領域の名前
        static constexpr const char* UNKNOWN_NAME = "unknown";  // 空中の名前

        //** Constructor **//
        Obstacle_2(const Segment_2 segment_2);
        Obstacle_2(const Segment_2 segment_2, 
                   const bool is_transparent, 
                   const bool is_navigable, 
                   const bool is_domain=false, 
                   const std::string name=UNKNOWN_NAME);
        Obstacle_2(const Segment_2 segment_2, 
                   const bool is_transparent, 
                   const bool is_navigable, 
                   std::unordered_set<std::shared_ptr<Edge_2>>&& x_edge_ptrs, 
                   const bool is_domain=false,
                   const std::string name=UNKNOWN_NAME);

        //** Getter **//
        bool get_is_transparent() const ;
        bool get_is_navigable() const ;
        std::unordered_set<std::shared_ptr<Edge_2>> get_x_edge_ptrs() const ;
        std::string get_name() const;

        //** Setter **//
        void set_is_transparent(bool is_transparent);
        void set_is_navigable(bool is_navigable);
        void set_x_edge_ptrs(std::unordered_set<std::shared_ptr<Edge_2>>&& x_edge_ptrs);
        void set_name(std::string name);
        
        //** Construction Method **//
        /*************************************************
         * @brief 多角形を線分に分解して障害物にする
         * 
         * @param polygon 
         * @param is_transparent 
         * @param is_navigable 
         * @param is_domain 
         * @param name 
         * @return std::vector<std::shared_ptr<Obstacle_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Obstacle_2>> convert_polygon(
            const Polygon_2& polygon,  
            const bool is_transparent, 
            const bool is_navigable, 
            const bool is_domain, 
            const std::string name=UNKNOWN_NAME
        );
        /*************************************************
         * @brief 交差する辺を追加する
         * 
         * @param edge_ptr 
         *************************************************/
        void insert_x_edge_ptr(std::shared_ptr<Edge_2> edge_ptr);
        /*************************************************
         * @brief 交差しなくなった辺を削除する
         * 
         * @param edge_ptr 
         *************************************************/
        void erase_x_edge_ptr(std::shared_ptr<Edge_2> edge_ptr);
        
        
        //** Geometric Method **//
        /*************************************************
         * @brief 辺と交差するか判定する
         * 
         * @param segment_2 
         * @return true 
         * @return false 
         *************************************************/
        bool is_intersecting(const Segment_2 segment_2) const;

        //** File IO Method **//
        /*************************************************
         * @brief 障害物を読み込む
         * 
         * @param abs_file_path 
         * @param is_transparent 
         * @param is_navigable 
         * @param is_domain 
         * @return std::vector<std::shared_ptr<Obstacle_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Obstacle_2>> read_obstacles(const std::string& abs_file_path, 
                                                                       const bool is_transparent, 
                                                                       const bool is_navigable, 
                                                                       const bool is_domain=false);
        /*************************************************
         * @brief 障害物を読み込む
         * 
         * @param abs_file_path 
         * @param is_domain 
         * @return std::vector<std::shared_ptr<Obstacle_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Obstacle_2>> read_obstacles(const std::string& abs_file_path, 
                                                                       const bool is_domain=false);
};

#endif // OBSTACLE_2