
// include header
#include "core/edge/edge_3_filter.h"

//** Constructor **//
Edge_3_Filter::Edge_3_Filter(const Net_3& net_3, const size_t mode) : 
    net_3(net_3), 
    mode(mode) 
{}

//** Operator **//
bool Edge_3_Filter::operator()(const Net_3::edge_descriptor& edge) const {
    switch (mode) {
        case Net_3::MODE_VISIBILITY :
            return net_3[edge]->get_is_visible();
        case Net_3::MODE_ROUTE :
            return net_3[edge]->get_is_passable();
        default:
            throw std::runtime_error("Invalid mode" + std::to_string(mode) + ".\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

}