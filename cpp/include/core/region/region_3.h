

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;

class Region_3 : public Polyhedron_3 {};