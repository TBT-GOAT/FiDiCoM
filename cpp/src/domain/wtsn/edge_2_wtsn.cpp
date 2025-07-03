
// include header
#include "domain/wtsn/edge_2_wtsn.h"

//** static **//
double Edge_2_WTSN::activated_ratio = 0.5;

//** Constructor **//
Edge_2_WTSN::Edge_2_WTSN(std::shared_ptr<Node_2> source_ptr, 
                         std::shared_ptr<Node_2> target_ptr) : 
    Edge_2(source_ptr, target_ptr)
{}

//** Getter **//
double Edge_2_WTSN::get_weight_passability() const {
    return this->weight_passability_wtsn.calc_weight_for_pedestrian();
}
double Edge_2_WTSN::get_activated_ratio() {
    return activated_ratio;
}

//** Setter **//
void Edge_2_WTSN::set_weight_passability(const double weight) {
    Edge_2::set_weight_passability(weight);
    this->weight_passability_wtsn.set_init_weight(weight);
}
void Edge_2_WTSN::set_activated_ratio(const double activated_ratio) {
    Edge_2_WTSN::activated_ratio = activated_ratio;
}

//** Simulation Functions **//
void Edge_2_WTSN::initialize() {
    weight_passability_wtsn.initialize();
}

double Edge_2_WTSN::damp() {
    return weight_passability_wtsn.damp();
}

double Edge_2_WTSN::amp() {
    return weight_passability_wtsn.amp();
}  

void Edge_2_WTSN::fix() {
    weight_passability_wtsn.set_is_fixed(true);
}

Weight_Passability_WTSN Edge_2_WTSN::access_weight_passability_wtsn() const {
    return weight_passability_wtsn;
}

void Edge_2_WTSN::assign_weight_passability_wtsn() {
    Weight_Passability_WTSN default_weight_passability_wtsn {};
    this->weight_passability_wtsn = default_weight_passability_wtsn;
}

void Edge_2_WTSN::assign_weight_passability_wtsn(const Weight_Passability_WTSN weight_passability_wtsn) {
    this->weight_passability_wtsn = weight_passability_wtsn;
}

bool Edge_2_WTSN::is_activated() {
    
    if (access_weight_passability_wtsn().get_is_fixed()) {
        return true;
    }

    return weight_passability_wtsn.get_curr_count() 
            >
           activated_ratio * weight_passability_wtsn.get_damp_required_count(); 
           
}