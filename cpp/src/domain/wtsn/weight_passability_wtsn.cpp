
// include std libraries
#include <cmath>
#include <iostream>
#include <float.h>

// include header
#include "domain/wtsn/weight_passability_wtsn.h"

//** static **//
Pedestrian Weight_Passability_WTSN::pedestrian = Pedestrian();
double Weight_Passability_WTSN::global_init_weight = DBL_MAX;
double Weight_Passability_WTSN::kappa = 0.0;                     // デフォルトは重みを最大限考慮

//** Constructor **//
Weight_Passability_WTSN::Weight_Passability_WTSN() : 
    is_fixed(true), 
    curr_count(0.0), 
    amp_count(1.0), 
    amp_count_standadizer(1), 
    damp_required_count(1), 
    amp_required_count(1)
{

    // 重み1で変化しない
    double init_weight = 1.0;
    this->omega = init_weight / this->global_init_weight;

}

Weight_Passability_WTSN::Weight_Passability_WTSN(const double init_weight, 
                               const size_t amp_count_standadizer, 
                               const size_t damp_required_count, 
                               const size_t amp_required_count) :
    is_fixed(false),
    curr_count(0.0),
    amp_count(DAMP_COUNT * (damp_required_count / amp_required_count) / amp_count_standadizer),
    amp_count_standadizer(amp_count_standadizer),
    damp_required_count(damp_required_count),
    amp_required_count(amp_required_count)
{
    
    if (init_weight < CONV_WEIGHT) {
        throw std::runtime_error("Invalid inital weight (>= convergent weight (" + std::to_string(CONV_WEIGHT) + ")).\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    this->omega = init_weight / this->global_init_weight;

}

//** Getter **//
Pedestrian& Weight_Passability_WTSN::get_pedestrian() {
    return pedestrian;
}
bool Weight_Passability_WTSN::get_is_fixed() const {
    return is_fixed;
}
double Weight_Passability_WTSN::get_global_init_weight() {
    return global_init_weight;
}
double Weight_Passability_WTSN::get_init_weight() const {
    // global_init_weightの変更に追従
    return global_init_weight * ( (1 - kappa) * omega + kappa );
}
double Weight_Passability_WTSN::get_init_weight(double _kappa) const {
    // global_init_weightの変更に追従
    return global_init_weight * ( (1 - _kappa) * omega + _kappa );
}
double Weight_Passability_WTSN::get_conv_weight() const {
    return CONV_WEIGHT;
}
double Weight_Passability_WTSN::get_curr_weight() const {

    if (get_curr_count() == 0.0) {
        // 増幅しきっている
        return get_init_weight();
    } else if (get_curr_count() == damp_required_count) {
        // 減衰しきっている
        return get_conv_weight();
    } else {
        return sigmoid();
    }

}
double Weight_Passability_WTSN::get_kappa() const {
    return kappa;
}
double Weight_Passability_WTSN::get_curr_count() const {
    if (curr_count < 0.0 || curr_count > damp_required_count) {
        throw std::runtime_error("Invalid count (0 <= count <= " + std::to_string(damp_required_count) + ", but " + std::to_string(curr_count) + ").\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
    return curr_count;
}
double Weight_Passability_WTSN::get_damp_count() const {
    return DAMP_COUNT;
}
double Weight_Passability_WTSN::get_amp_count() const {
    return amp_count;
}
size_t Weight_Passability_WTSN::get_amp_count_standadizer() const {
    return amp_count_standadizer;
}
size_t Weight_Passability_WTSN::get_damp_required_count() const {
    return damp_required_count;
}
size_t Weight_Passability_WTSN::get_amp_required_count() const {
    return amp_required_count;
}
double Weight_Passability_WTSN::get_sigmoid_band() const {
    return sigmoid_band;
}

//** Setter **//
void Weight_Passability_WTSN::set_is_fixed(const bool is_fixed) {
    this->is_fixed = is_fixed;
}
void Weight_Passability_WTSN::set_global_init_weight(const double global_init_weight) {
    
    if (global_init_weight < CONV_WEIGHT) {
        throw std::runtime_error("Invalid global inital weight (>= convergent weight (" + std::to_string(CONV_WEIGHT) + ")).\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    Weight_Passability_WTSN::global_init_weight = global_init_weight;

}
void Weight_Passability_WTSN::set_init_weight(const double init_weight) {
    // 新たな init_weight による global_init_weight の更新
    if (this->global_init_weight < init_weight) {
        set_global_init_weight(init_weight);
    }
    
    this->omega = init_weight / global_init_weight;

}
void Weight_Passability_WTSN::set_kappa(const double kappa) {
    Weight_Passability_WTSN::kappa = kappa;
}
void Weight_Passability_WTSN::set_curr_count(const double curr_count) {
    this->curr_count = curr_count;
}
void Weight_Passability_WTSN::set_amp_count_standadizer(const size_t amp_count_standadizer) {
    this->amp_count_standadizer = amp_count_standadizer;
    this->amp_count = DAMP_COUNT * (damp_required_count / amp_required_count) / amp_count_standadizer;
}
void Weight_Passability_WTSN::set_damp_required_count(const size_t damp_required_count) {
    this->damp_required_count = damp_required_count;
    this->amp_count = DAMP_COUNT * (damp_required_count / amp_required_count) / amp_count_standadizer;
}
void Weight_Passability_WTSN::set_amp_required_count(const size_t amp_required_count) {
    this->amp_required_count = amp_required_count;
    this->amp_count = DAMP_COUNT * (damp_required_count / amp_required_count) / amp_count_standadizer;
}
void Weight_Passability_WTSN::set_sigmoid_band(const double sigmoid_band) {
    this->sigmoid_band = sigmoid_band;
}

//** Simulation Functions **//
double Weight_Passability_WTSN::sigmoid() const {
    /*
    sigmoid(x, x_shift, y_shift, x_expansion, y_expansion, reverse=1)
    = y_expansion / ( 1 + exp( (-1) * -( x - x_shift ) / x_expansion ) ) + y_shift
                                 ^
                                 x方向に反転（x (= curr_count) が増えるほど減衰）
    */
    return (get_init_weight() - CONV_WEIGHT) / ( 1 + std::exp( (curr_count - damp_required_count / 2) / (damp_required_count / sigmoid_band) ) ) + CONV_WEIGHT;
    //                          ^                                                     ^                      ^                                       ^
    //                          値域を引き伸ばす                                        countの切片            定義域を引き伸ばす                        weightの切片
}

void Weight_Passability_WTSN::initialize() {
    set_curr_count(0.0);
}

double Weight_Passability_WTSN::damp() {

    if (is_fixed) {
        return get_curr_count();
    }

    double next_count = curr_count + DAMP_COUNT;
    
    if (next_count > damp_required_count) {
        // 減衰しきっている
        set_curr_count(get_damp_required_count());
    } else {
        set_curr_count(next_count);
    }

    return get_curr_count();

}

double Weight_Passability_WTSN::amp() {
    
    if (is_fixed) {
        return get_curr_count();
    }
    
    double next_count = curr_count - amp_count;
    
    if (curr_count < 0) {
        // 増幅しきっている
        set_curr_count(0.0);
    } else {
        set_curr_count(next_count);
    }

    return get_curr_count();

}

double Weight_Passability_WTSN::get_weight_for(const Pedestrian pedestrian) const {
    return pedestrian.get_coef() * (get_curr_weight() - CONV_WEIGHT) + CONV_WEIGHT;
}

double Weight_Passability_WTSN::calc_weight_for_pedestrian() const {
    return get_weight_for(this->pedestrian);
}