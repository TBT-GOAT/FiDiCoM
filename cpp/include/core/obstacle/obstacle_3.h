/*************************************************
 * @file obstacle_3.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 3D obstacle
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

#ifndef OBSTACLE_3_H
#define OBSTACLE_3_H

// include standards
#include <iostream>
#include <string>
#include <vector>
#include <memory>

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment_3;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef CGAL::Triangle_3<K> Triangle_3;
typedef CGAL::Surface_mesh<K::Point_3> Surface_mesh;

// Edge_3を前方宣言
class Edge_3;

class Obstacle_3 : public Triangle_3 {
    private:
        bool is_transparent = false;                        // 見通せる障害物かどうか
        bool is_navigable = false;                          // 通れる障害物かどうか //? 見通せないのに通れる障害物？カーテン？
        bool is_domain = false;                             // 対象領域かどうか
        std::unordered_set<std::shared_ptr<Edge_3>> x_edge_ptrs;   // 障害物と交差しているエッジ
        std::string name;                                   // 名前
    public:
        static constexpr const char* DOMAIN_NAME = "domain";    // 対象領域の名前
        static constexpr const char* UNKNOWN_NAME = "unknown";  // 空中の名前

        //** Constructor **//
        Obstacle_3(const Triangle_3 triangle_3);
        Obstacle_3(const Triangle_3 triangle_3, 
                   const bool is_transparent, 
                   const bool is_navigable, 
                   const bool is_domain=false, 
                   const std::string name=UNKNOWN_NAME);
        Obstacle_3(const Triangle_3 triangle_3, 
                   const bool is_transparent, 
                   const bool is_navigable, 
                   std::unordered_set<std::shared_ptr<Edge_3>>&& x_edge_ptrs, 
                   const bool is_domain=false, 
                   const std::string name=UNKNOWN_NAME);

        //** Getter **//
        bool get_is_transparent() const ;
        bool get_is_navigable() const ;
        std::unordered_set<std::shared_ptr<Edge_3>> get_x_edge_ptrs() const ;
        std::string get_name() const;

        //** Setter **//
        void set_is_transparent(bool is_transparent);
        void set_is_navigable(bool is_navigable);
        void set_x_edge_ptrs(std::unordered_set<std::shared_ptr<Edge_3>>&& x_edge_ptrs);
        void set_name(std::string name);
        
        //** Construction Method **//
        static std::vector<std::shared_ptr<Obstacle_3>> convert_polyhedron(
            Polyhedron_3& polyhedron,  
            const bool is_transparent, 
            const bool is_navigable, 
            const bool is_domain, 
            const std::string name=UNKNOWN_NAME
        );
        static std::vector<std::shared_ptr<Obstacle_3>> convert_surface_mesh(
            Surface_mesh& surface_mesh,  
            const bool is_transparent, 
            const bool is_navigable, 
            const bool is_domain, 
            const std::string name=UNKNOWN_NAME
        );

        void insert_x_edge_ptr(std::shared_ptr<Edge_3> edge_ptr);
        void erase_x_edge_ptr(std::shared_ptr<Edge_3> edge_ptr);
        
        //** Geometric Method **//
        bool is_intersecting(const Segment_3 segment_3) const;

        //** File IO Method **//
        static std::vector<std::shared_ptr<Obstacle_3>> read_obstacles(const std::string& abs_file_path, 
                                                                       const bool is_transparent, 
                                                                       const bool is_navigable, 
                                                                       const bool is_domain);
};

#endif // OBSTACLE_3