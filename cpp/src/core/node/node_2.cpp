

// include header
#include "core/node/node_2.h"


//** Constructor **//
Node_2::Node_2(bool is_dummy) : 
    Point_2(), is_dummy(is_dummy)
{}

Node_2::Node_2(const double x, const double y, bool is_dummy) : 
    Point_2(x, y), is_dummy(is_dummy)
{}

Node_2::Node_2(const Point_2 p, bool is_dummy) : 
    Point_2(p), is_dummy(is_dummy)
{}

//** Geometric Method **//
