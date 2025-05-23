

// include header
#include "core/region/region_2.h"

// include CGAL
#include <CGAL/intersections.h>


//** Constructor **//
Region_2::Region_2(const Polygon_2 polygon_2) : 
    Polygon_2(polygon_2)
{}

Region_2::Region_2(const Polygon_2 polygon_2, 
         const double weight_visibility, 
         const double weight_passability) :
    Polygon_2(polygon_2), 
    weight_visibility(weight_visibility), 
    weight_passability(weight_passability)
{}

//** Getter **//
double Region_2::get_weight_visibility() const {
    return weight_visibility;
}

double Region_2::get_weight_passability() const {
    return weight_passability;
}


//** Setter **//
void Region_2::set_weight_visibility(const double weight_visibility) {
    this->weight_visibility = weight_visibility;
}

void Region_2::set_weight_passability(const double weight_passability) {
    this->weight_passability = weight_passability;
}

//** Construction Method **//
std::vector<std::shared_ptr<Region_2>> convert_polygons(const std::vector<Polygon_2>& polygons) {

    std::vector<std::shared_ptr<Region_2>> regions;
    for (size_t i {0}; i <  polygons.size(); ++i) {
        regions.emplace_back(std::make_shared<Region_2>(polygons.at(i)));
    } 

    return regions;

}

std::vector<std::shared_ptr<Region_2>> convert_polygons(const std::vector<Polygon_2>& polygons, 
                                                        const std::vector<double> weights_visibility, 
                                                        const std::vector<double> weights_passability) {

    if (polygons.size() != weights_visibility.size() || polygons.size() != weights_passability.size()) {
        throw std::runtime_error(
            "Input vectors with the same size.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    std::vector<std::shared_ptr<Region_2>> regions;
    for (size_t i {0}; i <  polygons.size(); ++i) {
        regions.emplace_back(std::make_shared<Region_2>(polygons.at(i), 
                                                        weights_visibility.at(i), 
                                                        weights_passability.at(i)));
    } 

    return regions;

}

//** Geometric Method **//
bool Region_2::is_intersecting(const Segment_2 segment_2) const {
    if ((*this).bounded_side(segment_2.source()) == CGAL::ON_BOUNDED_SIDE 
        || 
        (*this).bounded_side(segment_2.target()) == CGAL::ON_BOUNDED_SIDE) {
        // 端点のどちらかが領域内である場合
        return true;
    }

    for (auto edge = (*this).edges_begin(); edge != (*this).edges_end(); ++edge) {
        if (CGAL::do_intersect(*edge, segment_2)) {
            return true;
        }
    }

    return false;
    
}

//** File IO Method **//
std::vector<std::shared_ptr<Region_2>> Region_2::read_regions(const std::string& abs_file_path) {
    
}