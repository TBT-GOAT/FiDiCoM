
// include std libraries
#include <iostream>
#include <random>
#include <cmath>  // for exp

// include header
#include "domain/wtsn/pedestrian.h"

// include util
#include "core/util/random_engine.h"


//** Constructor **//
Pedestrian::Pedestrian(double sigma_coef): 
    sigma_coef(sigma_coef)
{
    this->dist = std::normal_distribution<double> (MU_COEF, sigma_coef);
}

//** Getter **//
double Pedestrian::get_coef() const {
    return coef;
}
double Pedestrian::get_sigma_coef() const {
    return sigma_coef;
}


//** Setter **//
void Pedestrian::set_coef(const double coef) {
    this->coef = coef;
}
void Pedestrian::set_sigma_coef(const double sigma_coef) {
    this->sigma_coef = sigma_coef;
    this->dist = std::normal_distribution<double> (this->MU_COEF, this->sigma_coef);
}


//** Simulation Functions **//
void Pedestrian::change_coef() {
    double new_coef = std::exp(dist(Random_Engine::get_engine()));
    set_coef(new_coef); 
}

void Pedestrian::change_pedestrian() {
    change_coef();
}
