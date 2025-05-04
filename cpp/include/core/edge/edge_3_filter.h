/*************************************************
 * @file edge_3_filter.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief filtering edge (defining if edge is connected) based on visibility or passability 
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

#ifndef EDGE_3_FILTER_H
#define EDGE_3_FILTER_H

// include boost library
#include <boost/graph/filtered_graph.hpp>

// include Net_2
#include "core/net/net_3.h"

class Edge_3_Filter {
    private:
        const Net_3& net_3;
        const size_t mode;
    
    public:
        //** Constructor **//
        Edge_3_Filter(const Net_3& net_3, const size_t mode);

        //** Operator **//
        /*************************************************
         * @brief モードに応じて有効な辺かどうかを判定する
         * 
         * @param edge 
         * @return true 
         * @return false 
         *************************************************/
        bool operator()(const Net_3::edge_descriptor& edge) const;

};

#endif // EDGE_3_FILTER_H