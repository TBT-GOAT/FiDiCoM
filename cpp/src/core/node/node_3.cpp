

// include header
#include "core/node/node_3.h"


//** Constructor **//
Node_3::Node_3() : 
    Point_3()
{}

Node_3::Node_3(const double x, const double y, const double z) : 
    Point_3(x, y, z) 
{}

Node_3::Node_3(const Point_3 p) : 
    Point_3(p)
{}

//** Geometric Method **//
