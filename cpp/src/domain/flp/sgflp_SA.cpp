

// include header
#include "domain/flp/sgflp_SA.h"

// include util
#include "core/util/random_engine.h"

// include STL
#include <chrono>

//** Constructor **//
SGFLP_SA::SGFLP_SA(Net_SGFLP net_sgflp, 
                   double jump_rate, 
                   double sampling_rate,
                   bool knowing_closest_anchor)
         : net_sgflp(std::move(net_sgflp)),
           jump_rate(jump_rate),
           sampling_rate(sampling_rate),
           knowing_closest_anchor(knowing_closest_anchor) {}

//** Objective Function Method **//
double SGFLP_SA::evaluate_function(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs, 
    const size_t mode,
    bool evaluate_all_demands
) {
    // 割当の再構築
    const auto& new_facilities = facilities_and_signs.first;
    const auto& new_signs = facilities_and_signs.second;
    // this->net_sgflp.rebuild_facility_and_sign_setting(new_facilities, new_signs); // 再構成バージョン。確実だが遅い。
    this->net_sgflp.update_facility_and_sign_setting(new_facilities, new_signs);

    // 目的関数の計算
    double objective;

    // 需要点のサンプリング
    const auto& demands = this->net_sgflp.get_demands();

    std::vector<Net_2::vertex_descriptor> evaluated_demands;

    if (evaluate_all_demands || sampling_rate >= 1.0) {
        evaluated_demands = demands;
    } else {
        evaluated_demands.reserve(
            static_cast<size_t>(demands.size() * sampling_rate));

        std::sample(
            demands.begin(),
            demands.end(),
            std::back_inserter(evaluated_demands),
            static_cast<size_t>(demands.size() * sampling_rate),
            Random_Engine::get_engine());
    }

    // 評価
    if (mode == MODE_MINSUM) {
        objective = 0.0;
        for (const auto& demand : evaluated_demands) {
            objective += this->net_sgflp.calculate_cost(demand, this->knowing_closest_anchor).second;
        }

        // double full_objective = 0.0;
        // for (const auto& demand : demands) {
        //     full_objective += this->net_sgflp.calculate_cost(demand).second;
        // }

        // std::cout << "Evaluated objective: " << objective / sampling_rate << std::endl;
        // std::cout << "Full objective: " << full_objective << std::endl;
        // std::cout << "Error rate: " << std::abs(objective / sampling_rate - full_objective) / full_objective * 100 << "%" << std::endl;
        // std::cout << std::endl;

    } else if (mode == MODE_MINMAX) {
        objective = 0.0;
        std::vector<double> costs;
        for (const auto& demand : evaluated_demands) {
            costs.push_back(this->net_sgflp.calculate_cost(demand, this->knowing_closest_anchor).second);
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

std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> SGFLP_SA::generate_neighbor_function_without_jump(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs
) const {
    std::vector<Net_2::vertex_descriptor> demands = this->net_sgflp.get_demands();
    
    // facilitiesかsignsからひとつ選んで隣接頂点に移動する
    std::vector<Net_2::vertex_descriptor> facilities = facilities_and_signs.first;
    std::vector<Net_2::vertex_descriptor> signs = facilities_and_signs.second;
    const std::vector<Net_2::vertex_descriptor> anchors = this->net_sgflp.get_anchors();
    size_t facility_num = facilities.size();
    size_t sign_num = signs.size();
    size_t elem_num = facility_num + sign_num;

    const auto available_demands_for = [&](Net_2::vertex_descriptor moving_entity) {
        std::vector<Net_2::vertex_descriptor> available_demands;
        available_demands.reserve(demands.size());
        for (const auto& demand : demands) {
            const bool occupied_by_other_entity =
                (std::find(facilities.begin(), facilities.end(), demand) != facilities.end() && demand != moving_entity) ||
                (std::find(signs.begin(), signs.end(), demand) != signs.end() && demand != moving_entity) ||
                (std::find(anchors.begin(), anchors.end(), demand) != anchors.end());
            if (!occupied_by_other_entity) {
                available_demands.push_back(demand);
            }
        }
        return available_demands;
    };

    if (elem_num == 0) {
        return std::make_pair(facilities, signs);
    }

    std::uniform_int_distribution<> elem_dist(0, elem_num - 1);
    size_t target_elem_index = elem_dist(Random_Engine::get_engine());

    size_t target_facility_index;
    size_t target_sign_index;
    if (target_elem_index < facility_num) {
        // facilitiesから選ぶ
        target_facility_index = target_elem_index; 
        Net_2::vertex_descriptor target_facility = facilities.at(target_facility_index);
        std::vector<Net_2::vertex_descriptor> available_demands = available_demands_for(target_facility);
        if (available_demands.empty()) {
            return std::make_pair(facilities, signs);
        }
        
        std::vector<Net_2::vertex_descriptor> adjacents_to_target_facility;
        Net_2::adjacency_iterator adj_vit, adj_vit_end;
        for (
            boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_facility, *(this->net_sgflp.net_ptr)); 
            adj_vit != adj_vit_end; 
            ++adj_vit
        ) {
            // ダミーでない需要点を選ぶ
            if (find_index(available_demands, *adj_vit) != -1 && (*(this->net_sgflp.net_ptr))[*adj_vit]->get_is_dummy() == false) {
                adjacents_to_target_facility.push_back(*adj_vit);
            }
        }

        if (adjacents_to_target_facility.empty()) {
            // fallback: 他エンティティが使用していない需要点から選ぶ
            adjacents_to_target_facility = available_demands;
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
        std::vector<Net_2::vertex_descriptor> available_demands = available_demands_for(target_sign);
        if (available_demands.empty()) {
            return std::make_pair(facilities, signs);
        }

        std::vector<Net_2::vertex_descriptor> adjacents_to_target_sign;
        Net_2::adjacency_iterator adj_vit, adj_vit_end;
        for (
            boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_sign, *(this->net_sgflp.net_ptr)); 
            adj_vit != adj_vit_end; 
            ++adj_vit
        ) {
            // ダミーでない需要点を選ぶ
            if (find_index(available_demands, *adj_vit) != -1 && (*(this->net_sgflp.net_ptr))[*adj_vit]->get_is_dummy() == false) {
                adjacents_to_target_sign.push_back(*adj_vit);
            }
        }

        if (adjacents_to_target_sign.empty()) {
            // fallback: 他エンティティが使用していない需要点から選ぶ
            adjacents_to_target_sign = available_demands;
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

std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> SGFLP_SA::generate_neighbor_function_with_jump(
    const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs
) const {
    std::vector<Net_2::vertex_descriptor> demands = this->net_sgflp.get_demands();
    
    // facilitiesかsignsからひとつ選んで移動する
    std::vector<Net_2::vertex_descriptor> facilities = facilities_and_signs.first;
    std::vector<Net_2::vertex_descriptor> signs = facilities_and_signs.second;
    const std::vector<Net_2::vertex_descriptor> anchors = this->net_sgflp.get_anchors();
    size_t facility_num = facilities.size();
    size_t sign_num = signs.size();
    size_t elem_num = facility_num + sign_num;

    const auto available_demands_for = [&](Net_2::vertex_descriptor moving_entity) {
        std::vector<Net_2::vertex_descriptor> available_demands;
        available_demands.reserve(demands.size());
        for (const auto& demand : demands) {
            const bool occupied_by_other_entity =
                (std::find(facilities.begin(), facilities.end(), demand) != facilities.end() && demand != moving_entity) ||
                (std::find(signs.begin(), signs.end(), demand) != signs.end() && demand != moving_entity) ||
                (std::find(anchors.begin(), anchors.end(), demand) != anchors.end());
            if (!occupied_by_other_entity) {
                available_demands.push_back(demand);
            }
        }
        return available_demands;
    };

    if (elem_num == 0) {
        return std::make_pair(facilities, signs);
    }

    std::uniform_int_distribution<> elem_dist(0, elem_num - 1);
    size_t target_elem_index = elem_dist(Random_Engine::get_engine());

    std::uniform_real_distribution<> jump_dist(0.0, 1.0);
    double jump_rate_sample = jump_dist(Random_Engine::get_engine());

    size_t target_facility_index;
    size_t target_sign_index;
    Net_2::vertex_descriptor swapped_vertex;
    if (target_elem_index < facility_num) {
        // facilitiesから選ぶ
        target_facility_index = target_elem_index; 
        Net_2::vertex_descriptor target_facility = facilities.at(target_facility_index);
        std::vector<Net_2::vertex_descriptor> available_demands = available_demands_for(target_facility);
        if (available_demands.empty()) {
            return std::make_pair(facilities, signs);
        }

        if (jump_rate_sample < jump_rate) {
            // 需要点から選ぶ
            std::uniform_int_distribution<> demand_dist(0, available_demands.size() - 1);
            size_t target_demand_index = demand_dist(Random_Engine::get_engine());
            swapped_vertex = available_demands.at(target_demand_index);
        } else {
            // 隣接頂点だけから選ぶ
            std::vector<Net_2::vertex_descriptor> adjacents_to_target_facility;
            Net_2::adjacency_iterator adj_vit, adj_vit_end;
            for (
                boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_facility, *(this->net_sgflp.net_ptr)); 
                adj_vit != adj_vit_end; 
                ++adj_vit
            ) {
                // ダミーでない需要点を選ぶ
                if (find_index(available_demands, *adj_vit) != -1 && (*(this->net_sgflp.net_ptr))[*adj_vit]->get_is_dummy() == false) {
                    adjacents_to_target_facility.push_back(*adj_vit);
                }
            }

            if (adjacents_to_target_facility.empty()) {
                // fallback: 他エンティティが使用していない需要点から選ぶ
                adjacents_to_target_facility = available_demands;
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
        std::vector<Net_2::vertex_descriptor> available_demands = available_demands_for(target_sign);
        if (available_demands.empty()) {
            return std::make_pair(facilities, signs);
        }

        if (jump_rate_sample < jump_rate) {
            // 需要点から選ぶ
            std::uniform_int_distribution<> demand_dist(0, available_demands.size() - 1);
            size_t target_demand_index = demand_dist(Random_Engine::get_engine());
            swapped_vertex = available_demands.at(target_demand_index);
        } else {
            // 隣接頂点だけから選ぶ
            std::vector<Net_2::vertex_descriptor> adjacents_to_target_sign;
            Net_2::adjacency_iterator adj_vit, adj_vit_end;
            for (
                boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_sign, *(this->net_sgflp.net_ptr)); 
                adj_vit != adj_vit_end; 
                ++adj_vit
            ) {
                // ダミーでない需要点を選ぶ
                if (find_index(available_demands, *adj_vit) != -1 && (*(this->net_sgflp.net_ptr))[*adj_vit]->get_is_dummy() == false) {
                    adjacents_to_target_sign.push_back(*adj_vit);
                }
            }

            if (adjacents_to_target_sign.empty()) {
                // fallback: 他エンティティが使用していない需要点から選ぶ
                adjacents_to_target_sign = available_demands;
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