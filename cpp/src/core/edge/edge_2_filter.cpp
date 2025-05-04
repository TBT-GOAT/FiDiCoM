
// include header
#include "core/edge/edge_2_filter.h"

//** Constructor **//
Edge_2_Filter::Edge_2_Filter(const Net_2& net_2, const size_t mode) : 
    net_2(net_2), 
    mode(mode) 
{}

//** Operator **//
bool Edge_2_Filter::operator()(const Net_2::edge_descriptor& edge) const {
    switch (mode) {
        case Net_2::MODE_VISIBILITY :
            return net_2[edge]->get_is_visible();
        case Net_2::MODE_ROUTE :
            return net_2[edge]->get_is_passable();
        default:
            throw std::runtime_error("Invalid mode" + std::to_string(mode) + ".\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

}