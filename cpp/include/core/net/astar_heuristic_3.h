

#ifndef ASTAR_HEURISTIC_3_H
#define ASTAR_HEURISTIC_3_H

// include std libraries
#include <cmath>  // std::sqrt

// include astar
#include <boost/graph/astar_search.hpp>

// include Net_3
#include "core/net/net_3.h"


class Astar_Heuristic_3 : public boost::astar_heuristic<Net_3, double> {
    public:
        Astar_Heuristic_3(const Net_3& net, Net_3::vertex_descriptor goal)
            : net(net), goal(goal) {}

        double operator()(Net_3::vertex_descriptor u) {
            const auto& p1_ptr = net[u];
            const auto& p2_ptr = net[goal];
            return std::sqrt(CGAL::squared_distance(*p1_ptr, *p2_ptr));
        }

    private:
        const Net_3& net;
        Net_3::vertex_descriptor goal;
};

struct Astar_Found_Goal_3 {}; // exception for termination

class Astar_Goal_Visitor_3 : public boost::default_astar_visitor {
    public:
        Astar_Goal_Visitor_3(Net_3::vertex_descriptor goal) : m_goal(goal) {}

        void examine_vertex(Net_3::vertex_descriptor vertex, const Net_3& net) {
            if(vertex == m_goal)
            throw Astar_Found_Goal_3();
        }
    private:
        Net_3::vertex_descriptor m_goal;
};


#endif // ASTAR_HEURISTIC_3_H