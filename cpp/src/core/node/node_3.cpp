

// include header
#include "core/node/node_3.h"


//** Constructor **//
Node_3::Node_3(bool is_dummy) : 
    Point_3(), is_dummy(is_dummy)
{}

Node_3::Node_3(const double x, const double y, const double z, bool is_dummy) : 
    Point_3(x, y, z), is_dummy(is_dummy)
{}

Node_3::Node_3(const Point_3 p, bool is_dummy) : 
    Point_3(p), is_dummy(is_dummy)
{}

//** Geometric Method **//
