

// include header
#include "core/weight_mapper/weight_mapper_2.h"

// include standard
#include <limits>

// include stl util
#include "core/util/std_vector_util.h"

//** Constructor **//
Weight_Mapper_2::Weight_Mapper_2(const Net_2& net_2, 
                                 size_t mode, 
                                 bool using_obstacle, 
                                 bool using_weight, 
                                 std::vector<Net_2::vertex_descriptor> prohibited_vertices) : 
    net_2(net_2), 
    mode(mode), 
    using_obstacle(using_obstacle), 
    using_weight(using_weight), 
    prohibited_vertices(prohibited_vertices) 
{}

//** Operator **//
double Weight_Mapper_2::operator()(const Net_2::edge_descriptor edge_desc) const {
    
    const std::shared_ptr<Edge_2> edge_ptr = net_2[edge_desc];

    // 禁止された頂点を端点に持つエッジを禁止する
    if (find_index(prohibited_vertices, boost::source(edge_desc, net_2)) != -1 || 
        find_index(prohibited_vertices, boost::target(edge_desc, net_2)) != -1) 
    {
        return std::numeric_limits<double>::infinity(); // 長さを無限大にして実質的に通れなくする     
    }
    
    switch (mode) {
        case Net_2::MODE_VISIBILITY:
            if (using_obstacle && !edge_ptr->get_is_visible()) {
                return std::numeric_limits<double>::infinity(); // 長さを無限大にして実質的に通れなくする
            } 
            return using_weight ? edge_ptr->get_weight_visibility() * edge_ptr->calc_length()
                                : edge_ptr->calc_length();
        
        case Net_2::MODE_ROUTE:
            if (using_obstacle && !edge_ptr->get_is_passable()) {
                return std::numeric_limits<double>::infinity(); // 長さを無限大にして実質的に通れなくする
            } 
            return using_weight ? edge_ptr->get_weight_passability() * edge_ptr->calc_length()
                                : edge_ptr->calc_length();

        default:
            throw std::runtime_error("Invalid mode" + std::to_string(mode) + ".\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));        
    }
}