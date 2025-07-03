

// include header
#include "domain/wtsn/region_2_wtsn.h"


//** Constructor **//
Region_2_WTSN::Region_2_WTSN(const Polygon_2 polygon_2, 
                             const Weight_Passability_WTSN weight_passability_wtsn) :
    Region_2(polygon_2), 
    weight_passability_wtsn(weight_passability_wtsn)
{}
Region_2_WTSN::Region_2_WTSN(const Weight_Passability_WTSN weight_passability_wtsn) : 
    Region_2(),
    weight_passability_wtsn(weight_passability_wtsn)
{}

//** Getter **//
double Region_2_WTSN::get_weight_passability() const {
    return weight_passability_wtsn.get_init_weight(0.0);
}
Weight_Passability_WTSN Region_2_WTSN::get_weight_passability_wtsn() const {
    return weight_passability_wtsn;
}


//** Setter **//
void Region_2_WTSN::set_weight_passability(const double weight_passability) {
    this->weight_passability_wtsn.set_init_weight(weight_passability);
}
void Region_2_WTSN::set_weight_passability_wtsn(const Weight_Passability_WTSN weight_passability_wtsny) {
    this->weight_passability_wtsn = weight_passability_wtsn;
}

//** Construction Method **//
std::vector<std::shared_ptr<Region_2_WTSN>> convert_polygons(const std::vector<Polygon_2>& polygons, 
                                                             const std::vector<Weight_Passability_WTSN> weights_passability_wtsn) {

    if (polygons.size() != weights_passability_wtsn.size()) {
        throw std::runtime_error(
            "Input vectors with the same size.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    std::vector<std::shared_ptr<Region_2_WTSN>> regions;
    for (size_t i {0}; i <  polygons.size(); ++i) {
        regions.emplace_back(std::make_shared<Region_2_WTSN>(polygons.at(i), 
                                                            weights_passability_wtsn.at(i)));
    } 

    return regions;

}

//** File IO Method **//
std::vector<std::shared_ptr<Region_2_WTSN>> Region_2_WTSN::read_regions(const std::string& abs_file_path) {
    
}