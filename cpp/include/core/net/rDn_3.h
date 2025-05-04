/*************************************************
 * @file rDn_3.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 3D random Delaunay network
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

#ifndef RDN_3_H
#define RDN_3_H

// include Net_3
#include "core/net/net_3.h"

// include CGAL library
#include <CGAL/Delaunay_triangulation_3.h>
typedef CGAL::Delaunay_triangulation_3<K, CGAL::Fast_location>  Delaunay_3;

#include <CGAL/Delaunay_triangulation_2.h>
typedef CGAL::Delaunay_triangulation_2<K> Delaunay_2;

class rDn_3 : public Net_3 {
    private:
        Delaunay_3 tessellation;

        //** Geometric Method **//
        /*************************************************
         * @brief 頂点に対応するVertex_handleを検索する
         * 
         * @param node_ptr 
         * @return Delaunay_2::Vertex_handle 
         *************************************************/
        Delaunay_3::Vertex_handle search_geom_vertex(const std::shared_ptr<Node_3> node_ptr) const;
        /*************************************************
         * @brief Vertex_handleに対応する頂点を検索する
         * 
         * @param vh 
         * @return rDn_2::vertex_descriptor 
         *************************************************/
        rDn_3::vertex_descriptor search_net_vertex(const Delaunay_3::Vertex_handle vh) const;

    public:

        //** Constructor **//
        using Net_3::Net_3;

        //** Network Method **//
        /*************************************************
         * @brief ランダムドロネー網を初期化する
         * 
         *************************************************/
        void initialize() override;
        /*************************************************
         * @brief 与えられた頂点でランダムドロネー網を初期化する
         * 
         * @param node_ptrs 
         *************************************************/
        void initialize(const std::vector<std::shared_ptr<Node_3>> node_ptrs) override;

        //** Geometric Method **//
        /*************************************************
         * @brief 頂点に紐づくボロノイセルを構成する
         * 
         * @param node_ptr 
         * @return Polyhedron_3 
         *************************************************/
        Polyhedron_3 build_cell(const std::shared_ptr<Node_3> node_ptr) const override;
        /*************************************************
         * @brief 与えられた点に最も近い頂点を探索する
         * 
         * @return std::shared_ptr<Node_2> 
         *************************************************/
        Net_3::vertex_descriptor find_nearest_node(Point_3 p) const override;

        //** File IO Method **//
        /*************************************************
         * @brief 頂点に紐づいたボロノイセルを書き出す
         * 
         * @param abs_file_path 
         *************************************************/
        void write_cells(const std::string& abs_file_path) const override;
        /*************************************************
         * @brief 頂点に紐づいたボロノイセルを書き出す
         * 
         * @param abs_file_path 
         * @param node_ptrs 
         *************************************************/
        void write_cells(const std::string& abs_file_path, const std::vector<std::shared_ptr<Node_3>> node_ptrs) const override;

}; 

#endif // RDN_3_H