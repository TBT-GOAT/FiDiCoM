

// include header
#include "domain/wtsn/simulator_wtsn.h"

// include util
#include "core/util/union_find.h"



//** Constructor **//
Simulator_WTSN::Simulator_WTSN(Polygon_2 domain, 
                               std::shared_ptr<rDn_2_WTSN> net_ptr, 
                               std::vector<Point_2> demand_points, 
                               std::vector<std::vector<double>> demand_matrix) :
    domain(domain), 
    net_ptr(net_ptr)
{
    // demand_nodes
    for (auto& demand_point : demand_points) {
        rDn_2_WTSN::vertex_descriptor nearest_vertex = this->net_ptr->find_nearest_node(demand_point);
        
        if (domain.bounded_side(*(*net_ptr)[nearest_vertex]) != CGAL::ON_UNBOUNDED_SIDE) {
            // 対称領域内に最近点を発見
            demand_nodes.push_back(nearest_vertex);
        } else {
            // 対称領域外に最近点を発見
            // 最近点に最も近いノードの中で対象領域内にあるものを代替
            std::vector<std::pair<rDn_2_WTSN::vertex_descriptor, double>> spt = 
            net_ptr->calculate_shortest_path_tree(nearest_vertex, 
                                                  Net_2::MODE_ROUTE, 
                                                  false, 
                                                  false);
            
            // 最短経路木上の距離の昇順でソート
            //!ユークリッド距離の順序とは一致しない
            std::sort(spt.begin(), spt.end(), [](const auto& a, const auto& b) {
                return a.second < b.second;
            });

            for (const auto& v_and_d : spt) {
                
                if (domain.bounded_side(*(*net_ptr)[v_and_d.first]) != CGAL::ON_UNBOUNDED_SIDE) {
                    demand_nodes.push_back(v_and_d.first);
                    break;
                }
            }

        }

    }

    // trans_prob_matrix
    trans_prob_matrix = normalize_matrix(demand_matrix);
}

//** Getter **//
std::shared_ptr<rDn_2_WTSN> Simulator_WTSN::get_net_ptr() const {
    return net_ptr;
}
std::vector<rDn_2_WTSN::vertex_descriptor> Simulator_WTSN::get_demand_nodes() const {
    return demand_nodes;
}
std::vector<std::vector<double>> Simulator_WTSN::get_trans_prob_matrix() const {
    return trans_prob_matrix;
}
std::set<rDn_2_WTSN::edge_descriptor> Simulator_WTSN::get_unsaturated_edges() const {
    return unsaturated_edges;
}
std::set<rDn_2_WTSN::edge_descriptor> Simulator_WTSN::get_activated_edges() const {
    return activated_edges;
}

//** Simulation Functions **//
std::vector<std::vector<double>> Simulator_WTSN::normalize_matrix(const std::vector<std::vector<double>>& matrix) {
    std::vector<std::vector<double>> normalized_matrix = matrix;

    // 和を求める
    double sum {0.0};
    for (size_t i {0}; i < matrix.size(); ++i) {
        for (size_t j {0}; j < matrix.at(i).size(); ++j) {
            sum += matrix.at(i).at(j);
        }
    }

    // 和を1に標準化
    for (size_t i {0}; i < matrix.size(); ++i) {
        for (size_t j {0}; j < matrix.at(i).size(); ++j) {
            normalized_matrix.at(i).at(j) /= sum;
        }
    }

    return normalized_matrix;

}

std::pair<rDn_2_WTSN::vertex_descriptor, rDn_2_WTSN::vertex_descriptor> Simulator_WTSN::choose_OD() {
    double prob = this->dist(Random_Engine::get_engine());
    double curr_val {0.0}; // 移動確率行列の要素和
    rDn_2_WTSN::vertex_descriptor O;
    rDn_2_WTSN::vertex_descriptor D;
    for (size_t i {0}; i < trans_prob_matrix.size(); ++i) {
        for (size_t j {0}; j < trans_prob_matrix.size(); ++j) {
            curr_val += trans_prob_matrix.at(i).at(j);

            if (curr_val > prob) {
                O = demand_nodes.at(i);
                D = demand_nodes.at(j);
                return std::make_pair(O, D);
            }

        }
    }

    throw std::runtime_error(
        "Could not choose OD.\n"
        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
    );

}

std::deque<rDn_2_WTSN::vertex_descriptor> Simulator_WTSN::calc_pedestrian_path(const rDn_2_WTSN::vertex_descriptor O, 
                                                                               const rDn_2_WTSN::vertex_descriptor D) const {
    std::deque<rDn_2_WTSN::vertex_descriptor> shortest_path;
    std::vector<std::pair<rDn_2_WTSN::vertex_descriptor, double>> shortest_path_tree;
    
    shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(O, 
                                                                     Net_2::MODE_ROUTE, 
                                                                     true, 
                                                                     true);

    if (shortest_path_tree.at(D).second == std::numeric_limits<double>::max()) {
        // 到達不可能
        return shortest_path;
    }

    for (auto vertex=D; vertex != O; vertex = shortest_path_tree.at(vertex).first) {
        shortest_path.push_front(vertex);
    }

    shortest_path.push_front(O);

    return shortest_path;

}

void Simulator_WTSN::damp(std::deque<rDn_2_WTSN::vertex_descriptor> pedestrian_path) {
    for (size_t i {0}; i < pedestrian_path.size() - 1; ++i) {
        std::pair<rDn_2_WTSN::edge_descriptor, bool> edge_existance;
        edge_existance = boost::edge(pedestrian_path.at(i), 
                                     pedestrian_path.at(i + 1), 
                                     *get_net_ptr());
        
        if (!edge_existance.second) {
            throw std::runtime_error("Invalid adjacency (" + std::to_string(pedestrian_path.at(i)) + " and " + std::to_string(pedestrian_path.at(i + 1)) + ").\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        // 重みを減衰
        std::shared_ptr<Edge_2_WTSN> edge_ptr = std::dynamic_pointer_cast<Edge_2_WTSN>((*get_net_ptr())[edge_existance.first]);
        edge_ptr->damp();

        // 重みに変化のあるエッジとして登録
        unsaturated_edges.insert(edge_existance.first);

    }
}

void Simulator_WTSN::amp() {
    
    for (auto it = unsaturated_edges.begin(); it != unsaturated_edges.end(); ) {
        auto edge = *it;
        // 重みに変化のあるエッジだけを対象に重みを増幅させれば良い
        // 他のエッジは重みがCONV_WEIGHTに収束済み

        // 重みを増幅
        double count = std::dynamic_pointer_cast<Edge_2_WTSN>((*get_net_ptr())[edge])->amp(); // 増幅後のカウント

        if (count == 0.0) {
            // 重みが増幅しきったエッジを登録から削除
            it = unsaturated_edges.erase(it);  // erase() は次のイテレータを返す
        } else {
            ++it;
        }
    }

}

void Simulator_WTSN::update_activated_edges() {
    // 一旦空にする
    activated_edges = {};
    
    // エッジを走査
    rDn_2_WTSN::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = boost::edges(*net_ptr); eit != eit_end; ++eit) {
        std::shared_ptr<Edge_2_WTSN> edge_ptr = std::dynamic_pointer_cast<Edge_2_WTSN>((*get_net_ptr())[*eit]);
        // 活性化判定
        if (edge_ptr->is_activated()) {
            activated_edges.insert(*eit);
        }
    }
        
}

bool Simulator_WTSN::is_connecting_all_demand_nodes() {
    
    if (activated_edges.empty()) {
        return false;
    }

    Union_Find<rDn_2_WTSN::vertex_descriptor> union_find; // 連結する部分グラフを数え上げる構造
    std::unordered_set<rDn_2_WTSN::vertex_descriptor> included_vertices; // 活性化したエッジに含まれる頂点の集合

    // 連結する部分グラフを構成
    for (auto& edge : activated_edges) {
        // 頂点の登録
        rDn_2_WTSN::vertex_descriptor s = boost::source(edge, *get_net_ptr());
        rDn_2_WTSN::vertex_descriptor t = boost::target(edge, *get_net_ptr());
        union_find.initialize(s);
        union_find.initialize(t);
        included_vertices.insert(s);
        included_vertices.insert(t);

        // エッジの両端点をunion
        union_find.unite(s, t);
    }

    // 移動の需要点がすべてひとつのネットワークに含まれているかチェック
    rDn_2_WTSN::vertex_descriptor one_root = union_find.find(demand_nodes.at(0));
    for (auto& demand_node : demand_nodes) {
        if (union_find.find(demand_node) == demand_node) {
            // 活性化されたエッジに移動の需要点が含まれていない
            return false;
        } else if (union_find.find(demand_node) != one_root) {
            // 移動の需要点が含まれる集合が一致しない
            return false;
        }
    }

    return true;

}

void Simulator_WTSN::setup(const size_t threshold_duration, 
                           const double max_iteration) {
    this->threshold_duration = threshold_duration;
    this->max_iteration = max_iteration;
}

void Simulator_WTSN::run() {
    size_t OD_num = demand_nodes.size() * demand_nodes.size(); // 始点と終点の組み合わせの数

    size_t iter {0};
    size_t non_growth_duration {0};
    double prev_total_edge_length {0.0};

    while (non_growth_duration < threshold_duration 
            && 
           iter < max_iteration) 
    {
        // 歩行者を変更
        Weight_Passability_WTSN::get_pedestrian().change_pedestrian();

        // 歩行軌跡を計算
        std::pair<rDn_2_WTSN::vertex_descriptor, rDn_2_WTSN::vertex_descriptor> OD = choose_OD();
        rDn_2_WTSN::vertex_descriptor O = OD.first;
        rDn_2_WTSN::vertex_descriptor D = OD.second;
        std::deque<rDn_2_WTSN::vertex_descriptor> pedestrian_path = calc_pedestrian_path(O, D);

        // 歩行軌跡上のエッジの重みを減衰
        damp(pedestrian_path);

        // エッジの重みを増幅
        amp();

        // 活性化したエッジを更新
        update_activated_edges();

        // 結果
        bool connection = is_connecting_all_demand_nodes();
        double curr_total_edge_length {0.0};
        for (auto& edge : activated_edges) {
            curr_total_edge_length += (*get_net_ptr())[edge]->calc_length();
        }

        // 更新
        if (connection // 連結である
             && // かつ
            curr_total_edge_length <= prev_total_edge_length // ネットワークが成長していない
        ) {
            non_growth_duration += 1;
        }
        prev_total_edge_length = curr_total_edge_length;
        iter += 1;

    }

}