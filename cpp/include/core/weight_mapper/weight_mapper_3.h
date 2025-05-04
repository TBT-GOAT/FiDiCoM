/*************************************************
 * @file weight_mapper_3.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief mapper weighting 3D edge based on visibility and passability
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

#ifndef WEIGHT_MAPPER_3_H
#define WEIGHT_MAPPER_3_H

// include boost library
#include <boost/property_map/property_map.hpp>

// include network
#include "core/edge/edge_3.h"
#include "core/net/net_3.h"

class Weight_Mapper_3 {
    private:
        const Net_3& net_3;
        size_t mode = Net_3::MODE_ROUTE;
        bool using_obstacle;    // true: length = std::numeric_limits<double>::infinity()
        bool using_weight;      // true: weight*length, false: length
        std::vector<Net_3::vertex_descriptor> prohibited_vertices;

    public:
        using key_type = std::shared_ptr<Edge_3>;
        using value_type = double;
        using reference = double;
        using category = boost::readable_property_map_tag;

        //** Constructor **//
        Weight_Mapper_3(const Net_3& net_3, 
                        size_t mode, 
                        bool using_obstacle=true, 
                        bool using_weight=true, 
                        std::vector<Net_3::vertex_descriptor> prohibited_vertices={});

        //** Operator **//
        double operator()(const Net_3::edge_descriptor edge_desc) const;
};

#endif // WEIGHT_MAPPER_3_H