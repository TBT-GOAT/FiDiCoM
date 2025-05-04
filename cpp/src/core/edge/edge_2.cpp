

// include header
#include "core/edge/edge_2.h"

// include Obstacle_2
#include "core/obstacle/obstacle_2.h"

//** Constructor **//
Edge_2::Edge_2(std::shared_ptr<Node_2> source_ptr, std::shared_ptr<Node_2> target_ptr) : 
    Segment_2(*source_ptr, *target_ptr),
    source_ptr(source_ptr),
    target_ptr(target_ptr)
{}

//** Getter **//
std::shared_ptr<Node_2> Edge_2::get_source_ptr() const {
    return source_ptr;
}

std::shared_ptr<Node_2> Edge_2::get_target_ptr() const {
    return target_ptr;
}

bool Edge_2::get_is_visible() const {
    return is_visible;
}

bool Edge_2::get_is_passable() const {
    return is_passable;
}

bool Edge_2::get_is_dummy() const {
    return is_dummy;
}

double Edge_2::get_weight_visibility() const {
    return weight_visibility;
}

double Edge_2::get_weight_passability() const {
    return weight_passability;
}

std::unordered_set<std::shared_ptr<Obstacle_2>> Edge_2::get_x_obstacle_ptrs() const {
    return x_obstacle_ptrs;
}

//** Setter **//
void Edge_2::set_source_ptr(const std::shared_ptr<Node_2> source_ptr) {
    this->source_ptr = source_ptr;
}

void Edge_2::set_target_ptr(const std::shared_ptr<Node_2> target_ptr) {
    this->target_ptr = target_ptr;
}

void Edge_2::set_is_visible(const bool is_visible) {
    this->is_visible = is_visible;
}

void Edge_2::set_is_passable(const bool is_passable) {
    this->is_passable = is_passable;
}

void Edge_2::set_is_dummy(const bool is_dummy) {
    this->is_dummy = is_dummy;
}

void Edge_2::set_weight_visibility(const double weight) {
    this->weight_visibility = weight;
}

void Edge_2::set_weight_passability(const double weight) {
    this->weight_passability = weight;
}

void Edge_2::set_x_obstacle_ptrs(std::unordered_set<std::shared_ptr<Obstacle_2>>&& x_obstacle_ptrs) {
    this->x_obstacle_ptrs = std::move(x_obstacle_ptrs);
}

//** Construction Method **//
void Edge_2::insert_x_obstacle_ptr(std::shared_ptr<Obstacle_2> obstacle_ptr) {
    this->x_obstacle_ptrs.insert(obstacle_ptr);
}

void Edge_2::erase_x_obstacle_ptr(std::shared_ptr<Obstacle_2> obstacle_ptr) {
    this->x_obstacle_ptrs.erase(obstacle_ptr);
}

//** Geometric Method **//
double Edge_2::calc_length() const {
    
    if (is_dummy) {
        return 0.0;
    }
    
    return std::sqrt(CGAL::squared_distance(*source_ptr, *target_ptr));
    
}