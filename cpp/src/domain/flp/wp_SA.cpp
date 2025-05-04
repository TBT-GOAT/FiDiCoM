

// include header
#include "domain/flp/wp_SA.h"

// include STL
#include <chrono>


//** Constructor **//
WP_SA::WP_SA(Net_WP net_wp) : net_wp(std::move(net_wp)) {}

//** Objective Function Method **//
double WP_SA::evaluate_function(
    const std::vector<Net_2::vertex_descriptor> facilities, 
    const size_t mode
) {
    // ネットワークの更新
    this->net_wp.clear_tree();
    
    // サービス供給点の変更を記録
    //!generate_neighbor_functionの仕様により，原理的にはひとつしか変更はない
    std::vector<std::pair<Net_2::vertex_descriptor, Net_2::vertex_descriptor>> updated_facility_vertex_pairs {};
    for (size_t i {0}; i < this->net_wp.get_facilities().size(); ++i) {
        if (this->net_wp.get_facilities().at(i) != facilities.at(i)) {
            updated_facility_vertex_pairs.push_back(
                std::make_pair(this->net_wp.get_facilities().at(i), facilities.at(i))
            );
        }
    }

    for (const auto& updated_facility_vertex_pair : updated_facility_vertex_pairs) {
        this->net_wp.update_dummy_vertex_facilities(updated_facility_vertex_pair.first, updated_facility_vertex_pair.second);
    }

    this->net_wp.set_facilities(facilities);

    this->net_wp.build_tree();

    // 目的関数の計算
    double objective;
    
    if (mode == MODE_MINSUM) {
        objective = 0.0;
        for (const auto& demand : this->net_wp.get_demands()) {
            objective += this->net_wp.calculate_cost(demand);
        }
    } else if (mode == MODE_MINMAX) {
        objective = 0.0;
        std::vector<double> costs;
        for (const auto& demand : this->net_wp.get_demands()) {
            costs.push_back(this->net_wp.calculate_cost(demand));
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

std::vector<Net_2::vertex_descriptor> WP_SA::generate_neighbor_function(
    const std::vector<Net_2::vertex_descriptor> facilities
) const {
    std::vector<Net_2::vertex_descriptor> neighbor_facilities = facilities;
    std::vector<Net_2::vertex_descriptor> demands = this->net_wp.get_demands();
    
    // facilitiesからひとつ選んで隣接頂点に移動する
    size_t facility_num = neighbor_facilities.size();
    std::uniform_int_distribution<> f_dist(0, facility_num - 1);
    size_t target_facility_index = f_dist(Random_Engine::get_engine());

    Net_2::vertex_descriptor target_facility = neighbor_facilities.at(target_facility_index);
        
    std::vector<Net_2::vertex_descriptor> adjacents_to_target_facility;
    Net_2::adjacency_iterator adj_vit, adj_vit_end;
    for (boost::tie(adj_vit, adj_vit_end) = boost::adjacent_vertices(target_facility, *(this->net_wp.net_ptr)); 
         adj_vit != adj_vit_end; 
         ++adj_vit
    ) 
    {
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
    neighbor_facilities.at(target_facility_index) = target_adjacent;

    return neighbor_facilities;

}