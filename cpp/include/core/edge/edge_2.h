/*************************************************
 * @file edge.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D edge class
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

#ifndef EDGE_2_H
#define EDGE_2_H

// include standards
#include <iostream>
#include <vector>
#include <memory>

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;

// include Node_2
#include "core/node/node_2.h"

// Obstacle_2を前方宣言
class Obstacle_2;

class Edge_2 : public Segment_2{
    private:
        std::vector<std::shared_ptr<Segment_2>> obstacle_ptrs;  // エッジと交差する障害物
                                                                //! ひとつのエッジは複数の障害物と交差する可能性がある
        std::shared_ptr<Node_2> source_ptr; // 始点
        std::shared_ptr<Node_2> target_ptr; // 終点
        bool is_visible = true;             // 可視かどうか
        bool is_passable = true;            // 通過可能かどうか
        bool is_dummy = false;              // ダミーエッジかどうか
        double weight_visibility = 1;       // 可視性の重み
        double weight_passability = 1;      // 移動の重み
        std::unordered_set<std::shared_ptr<Obstacle_2>> x_obstacle_ptrs;    // エッジと交差している障害物
                                                                            //!Obstacle_2.x_edge_ptrsとの相互関係を維持するように管理を徹底

    public:
        //** Constructor **//
        Edge_2(std::shared_ptr<Node_2> source_ptr, std::shared_ptr<Node_2> target_ptr);

        //** Getter **//
        std::shared_ptr<Node_2> get_source_ptr() const;
        std::shared_ptr<Node_2> get_target_ptr() const;
        bool get_is_visible() const;
        bool get_is_passable() const;
        bool get_is_dummy() const;
        double get_weight_visibility() const;
        double get_weight_passability() const;
        std::unordered_set<std::shared_ptr<Obstacle_2>> get_x_obstacle_ptrs() const;

        //** Setter **//
        void set_source_ptr(const std::shared_ptr<Node_2> source_ptr);
        void set_target_ptr(const std::shared_ptr<Node_2> target_ptr);
        void set_is_visible(const bool is_visible);
        void set_is_passable(const bool is_passable);
        void set_is_dummy(const bool is_dummy);
        void set_weight_visibility(const double weight);
        void set_weight_passability(const double weight);
        void set_x_obstacle_ptrs(std::unordered_set<std::shared_ptr<Obstacle_2>>&& x_obstacle_ptrs);

        //** Construction Method **//
        /*************************************************
         * @brief 交差している障害物を追加する
         * 
         * @param obstacle_ptr 
         *************************************************/
        void insert_x_obstacle_ptr(std::shared_ptr<Obstacle_2> obstacle_ptr);
        /*************************************************
         * @brief 交差しなくなった障害物を削除する
         * 
         * @param obstacle_ptr 
         *************************************************/
        void erase_x_obstacle_ptr(std::shared_ptr<Obstacle_2> obstacle_ptr);

        //** Geometric Method **//
        /*************************************************
         * @brief 辺の長さを計算する
         * 
         * @return double 
         *************************************************/
        double calc_length() const;
        
};

#endif // EDGE_2_H