

// include header
#include "domain/flp/fslp_SA.h"

// include util
#include "core/util/random_engine.h"

// include STL
#include <chrono>

//** Constructor **//
FSLP_SA::FSLP_SA(Net_FSLP net_fslp) : net_fslp(std::move(net_fslp)) {}

//** Objective Function Method **//
double FSLP_SA::evaluate_function(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs, 
    const size_t mode
) {
    // ネットワークの更新
    this->net_fslp.clear_assignments();
    this->net_fslp.clear_trees();

    // サービス供給点，サインの変更を記録
    //!generate_neighbor_functionの仕様により，原理的にはひとつしか変更はない
    std::vector<std::pair<Net_2::vertex_descriptor, Net_2::vertex_descriptor>> updated_facility_vertex_pairs {};
    for (size_t i {0}; i < this->net_fslp.get_facilities().size(); ++i) {
        if (this->net_fslp.get_facilities().at(i) != facilities_and_signs.first.at(i)) {
            updated_facility_vertex_pairs.push_back(
                std::make_pair(this->net_fslp.get_facilities().at(i), facilities_and_signs.first.at(i))
            );
        }
    }

    for (const auto& updated_facility_vertex_pair : updated_facility_vertex_pairs) {
        this->net_fslp.update_dummy_vertex_facilities(updated_facility_vertex_pair.first, updated_facility_vertex_pair.second);
    }

    this->net_fslp.set_facilities(facilities_and_signs.first);
    
    std::vector<std::pair<Net_2::vertex_descriptor, Net_2::vertex_descriptor>> updated_sign_vertex_pairs {};
    for (size_t i {0}; i < this->net_fslp.get_signs().size(); ++i) {
        if (this->net_fslp.get_signs().at(i) != facilities_and_signs.second.at(i)) {
            updated_sign_vertex_pairs.push_back(
                std::make_pair(this->net_fslp.get_signs().at(i), facilities_and_signs.second.at(i))
            );
        }
    }

    for (const auto& updated_sign_vertex_pair : updated_sign_vertex_pairs) {
        this->net_fslp.update_dummy_vertex_signs(updated_sign_vertex_pair.first, updated_sign_vertex_pair.second);
    }

    this->net_fslp.set_signs(facilities_and_signs.second);

    
    this->net_fslp.build_trees();
    this->net_fslp.build_assignments();

    // 目的関数の計算
    double objective;

    if (mode == MODE_MINSUM) {
        objective = 0.0;
        for (const auto& demand : this->net_fslp.get_demands()) {
            objective += this->net_fslp.calculate_cost(demand);
        }
    } else if (mode == MODE_MINMAX) {
        objective = 0.0;
        std::vector<double> costs;
        for (const auto& demand : this->net_fslp.get_demands()) {
            costs.push_back(this->net_fslp.calculate_cost(demand));
        }
        objective = *std::max_element(costs.begin(), costs.end());
    } else {
        throw std::runtime_error(
            "Invalid mode.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    return objective;

}

std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> FSLP_SA::generate_neighbor_function_without_jump(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs
) const {
    std::vector<Net_2::vertex_descriptor> demands = this->net_fslp.get_demands();
    
    // facilitiesかsignsからひとつ選んで隣接頂点に移動する
    std::vector<Net_2::vertex_descriptor> facilities = facilities_and_signs.first;
    std::vector<Net_2::vertex_descriptor> signs = facilities_and_signs.second;
    size_t facility_num = facilities.size();
    size_t sign_num = signs.size();
    size_t elem_num = facility_num + sign_num;

    std::uniform_int_distribution<> elem_dist(0, elem_num - 1);
    size_t target_elem_index = elem_dist(Random_Engine::get_engine());

    size_t target_facility_index;
    size_t target_sign_index;
    if (target_elem_index < facility_num) {
        // facilitiesから選ぶ
        target_facility_index = target_elem_index; 
        Net_2::vertex_descriptor target_facility = facilities.at(target_facility_index);
        
        std::vector<Net_2::vertex_descriptor> adjacents_to_target_facility;
        Net_2::adjacency_iterator adj_vit, adj_vit_end;
        for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_facility, 
                *(this->net_fslp.net_ptr)); 
                adj_vit != adj_vit_end; ++adj_vit) {
                if (find_index(demands, *adj_vit) != -1) {
                    adjacents_to_target_facility.push_back(*adj_vit);
                }
        }

        // adjacents_to_target_facilityからランダムにひとつ選ぶ
        std::uniform_int_distribution<> adj_f_dist(0, adjacents_to_target_facility.size() - 1);
        size_t target_adjacent_index = adj_f_dist(Random_Engine::get_engine());
        Net_2::vertex_descriptor target_adjacent = adjacents_to_target_facility.at(target_adjacent_index);

        // 隣接頂点に移動
        // facilitiesのtarget_facilityをtarget_adjacentに置き換える
        facilities.at(target_facility_index) = target_adjacent;
        
    } else {
        // signsから選ぶ
        target_sign_index = target_elem_index - facility_num;
        Net_2::vertex_descriptor target_sign = signs.at(target_sign_index);

        std::vector<Net_2::vertex_descriptor> adjacents_to_target_sign;
        Net_2::adjacency_iterator adj_vit, adj_vit_end;
        for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_sign, 
                *(this->net_fslp.net_ptr)); 
                adj_vit != adj_vit_end; ++adj_vit) {
                if (find_index(demands, *adj_vit) != -1) {
                    adjacents_to_target_sign.push_back(*adj_vit);
                }
        }

        // adjacents_to_target_signからランダムにひとつ選ぶ
        std::uniform_int_distribution<> adj_s_dist(0, adjacents_to_target_sign.size() - 1);
        size_t target_adjacent_index = adj_s_dist(Random_Engine::get_engine());
        Net_2::vertex_descriptor target_adjacent = adjacents_to_target_sign.at(target_adjacent_index);

        // 隣接頂点に移動
        // signsのtarget_signをtarget_adjacentに置き換える
        signs.at(target_sign_index) = target_adjacent;
    }

    return std::make_pair(facilities, signs);

}

std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> FSLP_SA::generate_neighbor_function_with_jump(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs, 
    const double jump_rate
) const {
    std::vector<Net_2::vertex_descriptor> demands = this->net_fslp.get_demands();
    
    // facilitiesかsignsからひとつ選んで移動する
    std::vector<Net_2::vertex_descriptor> facilities = facilities_and_signs.first;
    std::vector<Net_2::vertex_descriptor> signs = facilities_and_signs.second;
    size_t facility_num = facilities.size();
    size_t sign_num = signs.size();
    size_t elem_num = facility_num + sign_num;

    std::uniform_int_distribution<> elem_dist(0, elem_num - 1);
    size_t target_elem_index = elem_dist(Random_Engine::get_engine());

    std::uniform_real_distribution<> jump_dist(0.0, 1.0);

    size_t target_facility_index;
    size_t target_sign_index;
    Net_2::vertex_descriptor swapped_vertex;
    if (target_elem_index < facility_num) {
        // facilitiesから選ぶ
        target_facility_index = target_elem_index; 
        Net_2::vertex_descriptor target_facility = facilities.at(target_facility_index);

        if (jump_rate < elem_dist(Random_Engine::get_engine())) {
            // 需要点から選ぶ
            std::uniform_int_distribution<> demand_dist(0, demands.size() - 1);
            size_t target_demand_index = demand_dist(Random_Engine::get_engine());
            swapped_vertex = demands.at(target_demand_index);
        } else {
            // 隣接頂点だけから選ぶ
            std::vector<Net_2::vertex_descriptor> adjacents_to_target_facility;
            Net_2::adjacency_iterator adj_vit, adj_vit_end;
            for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_facility, 
                    *(this->net_fslp.net_ptr)); 
                    adj_vit != adj_vit_end; ++adj_vit) {
                    if (find_index(demands, *adj_vit) != -1) {
                        adjacents_to_target_facility.push_back(*adj_vit);
                    }
            }

            // adjacents_to_target_facilityからランダムにひとつ選ぶ
            std::uniform_int_distribution<> adj_f_dist(0, adjacents_to_target_facility.size() - 1);
            size_t target_adjacent_index = adj_f_dist(Random_Engine::get_engine());
            swapped_vertex = adjacents_to_target_facility.at(target_adjacent_index);
        }

        // 移動
        // facilitiesのtarget_facilityをswapped_vertexに置き換える
        facilities.at(target_facility_index) = swapped_vertex;
        
    } else {
        // signsから選ぶ
        target_sign_index = target_elem_index - facility_num;
        Net_2::vertex_descriptor target_sign = signs.at(target_sign_index);

        if (jump_rate < elem_dist(Random_Engine::get_engine())) {
            // 需要点から選ぶ
            std::uniform_int_distribution<> demand_dist(0, demands.size() - 1);
            size_t target_demand_index = demand_dist(Random_Engine::get_engine());
            swapped_vertex = demands.at(target_demand_index);
        } else {
            // 隣接頂点だけから選ぶ
            std::vector<Net_2::vertex_descriptor> adjacents_to_target_sign;
            Net_2::adjacency_iterator adj_vit, adj_vit_end;
            for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_sign, 
                    *(this->net_fslp.net_ptr)); 
                    adj_vit != adj_vit_end; ++adj_vit) {
                    if (find_index(demands, *adj_vit) != -1) {
                        adjacents_to_target_sign.push_back(*adj_vit);
                    }
            }

            // adjacents_to_target_signからランダムにひとつ選ぶ
            std::uniform_int_distribution<> adj_s_dist(0, adjacents_to_target_sign.size() - 1);
            size_t target_adjacent_index = adj_s_dist(Random_Engine::get_engine());
            swapped_vertex = adjacents_to_target_sign.at(target_adjacent_index);
        }

        // 移動
        // signsのtarget_signをswapped_vertexに置き換える
        signs.at(target_sign_index) = swapped_vertex;
    }

    return std::make_pair(facilities, signs);

}