

#ifndef ASTAR_HEURISTIC_2_H
#define ASTAR_HEURISTIC_2_H

// include std libraries
#include <cmath>  // std::sqrt

// include astar
#include <boost/graph/astar_search.hpp>

// include Net_2
#include "core/net/net_2.h"


class Astar_Heuristic_2 : public boost::astar_heuristic<Net_2, double> {
    public:
        Astar_Heuristic_2(const Net_2& net, Net_2::vertex_descriptor goal)
            : net(net), goal(goal) {}

        double operator()(Net_2::vertex_descriptor u) {
            const auto& p1_ptr = net[u];
            const auto& p2_ptr = net[goal];
            return std::sqrt(CGAL::squared_distance(*p1_ptr, *p2_ptr));
        }

    private:
        const Net_2& net;
        Net_2::vertex_descriptor goal;
};

struct Astar_Found_Goal_2 {}; // exception for termination

class Astar_Goal_Visitor_2 : public boost::default_astar_visitor {
    public:
        using event_filter = boost::on_examine_vertex;

        Astar_Goal_Visitor_2(Net_2::vertex_descriptor goal) : m_goal(goal) {}

        void examine_vertex(Net_2::vertex_descriptor vertex, const Net_2& net) {
            if(vertex == m_goal)
            throw Astar_Found_Goal_2();
        }
    private:
        Net_2::vertex_descriptor m_goal;
};


#endif // ASTAR_HEURISTIC_2_H