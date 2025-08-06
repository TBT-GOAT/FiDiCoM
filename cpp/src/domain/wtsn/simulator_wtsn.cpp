

// include header
#include "domain/wtsn/simulator_wtsn.h"

// include util
#include "core/util/arithmetic_util.h"
#include "core/util/std_vector_util.h"
#include "core/util/union_find.h"

// include stl libraries
#include <chrono>

// include nlohmann-json
#include <nlohmann/json.hpp>
using json = nlohmann::json;


//** Constructor **//
Simulator_WTSN::Simulator_WTSN(const Polygon_2 domain, 
                               const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                               const std::vector<Point_2> demand_points, 
                               const std::vector<std::vector<double>> demand_matrix, 
                               const std::vector<Polygon_2> holes) :
    domain(domain), 
    holes(holes),
    net_ptr(net_ptr), 
    demand_points(demand_points)
{
    // demand_nodes
    find_demand_nodes(demand_points);

    // trans_prob_matrix
    if (demand_matrix.empty()) {
        // デフォルトの移動確率行列を設定
        size_t demand_num = demand_points.size();
        trans_prob_matrix = calc_uniform_trans_prob_matrix(demand_num);
    } else {
        // 入力された移動確率行列を設定
        if (demand_matrix.size() != demand_points.size() || 
            demand_matrix[0].size() != demand_points.size()) {
            throw std::runtime_error(
                "Input demand matrix size does not match the number of demand points.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }
        trans_prob_matrix = normalize_matrix(demand_matrix);
    }
    
}

Simulator_WTSN::Simulator_WTSN(const Polygon_2 domain, 
                               const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                               std::ifstream& input_file) : 
    domain(domain),
    net_ptr(net_ptr)
{
    json j;
    input_file >> j;

    // demand_nodes
    if (!j.contains("node_coordinates")) {
        throw std::runtime_error(
            "Input file does not contain 'node_coordinates'.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    } else {
        for (const auto& point : j["node_coordinates"]) {
            demand_points.emplace_back(point[0], point[1]);
        }

        find_demand_nodes(demand_points);
    }

    // trans_prob_matrix
    if (j.contains("weight_matrix")) {
        for (const auto& row : j["weight_matrix"]) {
            std::vector<double> vec_row;
            for (const auto& val : row) {
                vec_row.push_back(val);
            }
            trans_prob_matrix.push_back(vec_row);
        }

        trans_prob_matrix = normalize_matrix(trans_prob_matrix);
    } else {
        // デフォルトの移動確率行列を設定
        size_t demand_num = demand_points.size();
        trans_prob_matrix = calc_uniform_trans_prob_matrix(demand_num);
    }
    
}

Simulator_WTSN::Simulator_WTSN(const Polygon_2 domain, 
                               const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                               const std::string& input_file_path) : 
    domain(domain),
    net_ptr(net_ptr)
{
    std::ifstream input_file(input_file_path);
    if (!input_file.is_open()) {
        throw std::runtime_error("Could not open input file: " + input_file_path);
    }
    
    // コンストラクタのオーバーロード
    *this = Simulator_WTSN(domain, net_ptr, input_file);
    input_file.close();
}

//** Constructor Support Functions **//
bool Simulator_WTSN::is_in_domain(const Point_2& p) const {
    // 対象領域内に点があるか判定する
    if (domain.bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE) {

        for (const auto& hole : holes) {
            if (hole.bounded_side(p) == CGAL::ON_BOUNDED_SIDE) {
                // 穴の中にある場合はfalse
                return false;
            }
        }

        return true;
    
    } else {
        // 対象領域外にある場合はfalse
        return false;
    }
}

void Simulator_WTSN::find_demand_nodes(std::vector<Point_2> demand_points) {
    
    for (auto& demand_point : demand_points) {
        rDn_2_WTSN::vertex_descriptor nearest_vertex = this->net_ptr->find_nearest_node(demand_point);
        
        if (this->is_in_domain(*(*net_ptr)[nearest_vertex])) {
            // 対称領域内に最近点を発見
            // 穴の中にないかを確認
            //TODO Polygon_with_holes_2を使う
            for (const auto& hole : holes) {

            }
            
            demand_nodes.push_back(nearest_vertex);
            continue;

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
                
                if (this->is_in_domain(*(*net_ptr)[v_and_d.first])) {
                    demand_nodes.push_back(v_and_d.first);
                    break;
                }
            }

            continue;

        }

        // 対象領域内にノードが見つからなかった場合はエラー
        throw std::runtime_error(
            "Could not find any nodes in the domain for (" + std::to_string(demand_point.x()) + "," + std::to_string(demand_point.y()) + ").\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );

    }

    if (demand_nodes.size() != demand_points.size()) {
        // 対象領域内にあるノードの数が需要点の数と一致しない場合はエラー
        throw std::runtime_error(
            "Could not find all demand nodes.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

}

std::vector<std::vector<double>> Simulator_WTSN::calc_uniform_trans_prob_matrix(size_t demand_num) const {
    std::vector<std::vector<double>> uniform_matrix(demand_num, 
                                                    std::vector<double>(demand_num)
                                                    );
    for (size_t i {0}; i < demand_num; ++i) {
        for (size_t j {0}; j < demand_num; ++j) {
            if (i == j) {
                uniform_matrix[i][j] = 0.0; // 自分自身への移動確率は0
            } else {
                uniform_matrix[i][j] = 1.0 / (demand_num * (demand_num - 1)); // 等確率
            }
        }
    }

    return uniform_matrix;

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
std::set<rDn_2_WTSN::edge_descriptor> Simulator_WTSN::get_living_edges() const {
    return living_edges;
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
    std::deque<std::pair<rDn_2_WTSN::vertex_descriptor, double>> shortest_path;
    shortest_path = this->net_ptr->calculate_shortest_path(O, 
                                                           D, 
                                                           Net_2::MODE_ROUTE,
                                                           true, 
                                                           true);

    std::deque<rDn_2_WTSN::vertex_descriptor> pedestrian_path;
    for (auto v_and_d : shortest_path) {
        pedestrian_path.push_back(v_and_d.first);
    }

    return pedestrian_path;

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

void Simulator_WTSN::update_edge_sets() {
    // 一旦空にする
    this->activated_edges.clear();
    this->living_edges.clear();
    
    // エッジを走査
    rDn_2_WTSN::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = boost::edges(*net_ptr); eit != eit_end; ++eit) {
        std::shared_ptr<Edge_2_WTSN> edge_ptr = std::dynamic_pointer_cast<Edge_2_WTSN>((*get_net_ptr())[*eit]);
        
        // 活性化判定
        if (edge_ptr->is_activated()) {
            this->activated_edges.insert(*eit);
            // 動的であるか判定
            if (!edge_ptr->access_weight_passability_wtsn().get_is_fixed()) {
                this->living_edges.insert(*eit);
            }
        }

    }
        
}

bool Simulator_WTSN::is_connecting_all_demand_nodes() const {
    //TODO 正しく動いてなさそうな感じがする
    
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
    
    size_t iter {0};
    size_t non_growth_duration {0};
    double prev_total_living_edge_length {0.0};
    double curr_total_living_edge_length {0.0};
    bool connection = false;
    bool is_converged = false;

    while (non_growth_duration < threshold_duration 
            && 
           iter < max_iteration) 
    {
        
        std::cout << "\r" << iter << " / " << max_iteration << std::flush;
        
        // 歩行者を変更
        Weight_Passability_WTSN::get_pedestrian().change_pedestrian();

        // 歩行軌跡を計算
        std::pair<rDn_2_WTSN::vertex_descriptor, rDn_2_WTSN::vertex_descriptor> OD = choose_OD();
        rDn_2_WTSN::vertex_descriptor O = OD.first;
        rDn_2_WTSN::vertex_descriptor D = OD.second;
        std::deque<rDn_2_WTSN::vertex_descriptor> pedestrian_path = calc_pedestrian_path(O, D);

        if (pedestrian_path.empty()) {
            // 歩行軌跡が見つからない場合はスキップ
            std::cout << "No pedestrian path found." << std::endl;
            continue;
        }

        // 歩行軌跡上のエッジの重みを減衰
        damp(pedestrian_path);

        // エッジの重みを増幅
        amp();

        // エッジの集合を更新
        update_edge_sets();

        // 結果
        connection = is_connecting_all_demand_nodes();
        for (auto& edge : living_edges) {
            curr_total_living_edge_length += (*get_net_ptr())[edge]->calc_length();
        }

        // 収束判定
        if (
            connection // 連結である
              && // かつ
            curr_total_living_edge_length <= prev_total_living_edge_length // ネットワークが成長していない
        ) 
        {
            non_growth_duration++;
            is_converged = true;
        } else {
            non_growth_duration = 0;
            is_converged = false;
        }

        // 記録
        history.emplace_back(
            iter, 
            connection, 
            is_converged, 
            unsaturated_edges.size(), 
            activated_edges.size(), 
            living_edges.size(),
            curr_total_living_edge_length
        );

        // 更新
        prev_total_living_edge_length = curr_total_living_edge_length;
        curr_total_living_edge_length = 0.0;
        iter++;

    }

}

bool Simulator_WTSN::has_valid_result() const {
    return is_connecting_all_demand_nodes();
}

//** Analysis Functions **//
Net_2 Simulator_WTSN::build_wtsn_graph() const {
    Net_2 wtsn;
    std::unordered_map<rDn_2_WTSN::vertex_descriptor, Net_2::vertex_descriptor> vertex_map;

    // ノードのコピー
    for (auto edge : this->get_living_edges()) {
        auto src = boost::source(edge, *(this->net_ptr));
        auto tgt = boost::target(edge, *(this->net_ptr));
        if (vertex_map.find(src) == vertex_map.end()) {
            vertex_map[src] = boost::add_vertex((*(this->net_ptr))[src], wtsn);
        }
        if (vertex_map.find(tgt) == vertex_map.end()) {
            vertex_map[tgt] = boost::add_vertex((*(this->net_ptr))[tgt], wtsn);
        }
        
        std::shared_ptr<Edge_2> edge_ptr = std::make_shared<Edge_2>(wtsn[vertex_map[src]], wtsn[vertex_map[tgt]]);

        // エッジの重みを設定
        double w = std::dynamic_pointer_cast<Edge_2_WTSN>((*(this->get_net_ptr()))[edge])->access_weight_passability_wtsn().get_default_init_weight();
        edge_ptr->set_weight_passability(w);

        boost::add_edge(vertex_map[src], vertex_map[tgt], edge_ptr, wtsn);
    }

    // 移動需要点を復元
    for (size_t i {0}; i < demand_nodes.size(); ++i) {
        rDn_2_WTSN::vertex_descriptor original_demand_vertex = demand_nodes.at(i);
        wtsn[vertex_map[original_demand_vertex]] = std::make_shared<Node_2>(demand_points.at(i));
    }

    return wtsn;

}

Net_2 Simulator_WTSN::simplify_wtsn() const {
    
    //TODO 重み付きへの対応

    // WTSNのコピー
    Net_2 simplified_network = build_wtsn_graph();

    // 縮約
    bool changed = true;
    while (changed) {
        changed = false;
        Net_2::vertex_descriptor to_remove = Net_2::null_vertex();;

        // 削除すべき頂点の収集
        for (auto v : boost::make_iterator_range(boost::vertices(simplified_network))) {
            std::size_t deg = boost::degree(v, simplified_network);
            if (deg == 1 || deg == 2) {
                // 次数が 1 or 2 のとき
                if (find_index(demand_points, static_cast<Point_2>(*simplified_network[v])) == -1) {
                    // 移動需要点でないとき
                    // build_wtsn_graph() の出力は，移動需要点の位置を入力時に復元している
                    to_remove = v;
                    break;
                }
            }
        }

        // 削除
        if (to_remove == Net_2::null_vertex()) continue;
        std::size_t deg = boost::degree(to_remove, simplified_network);

        if (deg == 1) {
            auto nbr = *boost::adjacent_vertices(to_remove, simplified_network).first;
            boost::remove_edge(to_remove, nbr, simplified_network);
            boost::clear_vertex(to_remove, simplified_network);
            boost::remove_vertex(to_remove, simplified_network);
            changed = true;

        } else if (deg == 2) {
            auto it = boost::adjacent_vertices(to_remove, simplified_network);
            auto v1 = *it.first;
            auto v2 = *(++it.first);

            if (v1 != v2 && !boost::edge(v1, v2, simplified_network).second) {
                std::shared_ptr<Node_2> n1 = simplified_network[v1];
                std::shared_ptr<Node_2> n2 = simplified_network[v2];
                boost::add_edge(v1, v2, std::make_shared<Edge_2>(n1, n2), simplified_network);
            }

            boost::clear_vertex(to_remove, simplified_network);
            boost::remove_vertex(to_remove, simplified_network);
            changed = true;
        }

    }    

    return simplified_network;

}

double Simulator_WTSN::calc_total_length(Net_2& result_network, 
                                         bool is_weighted) const {
    double total_length;

    if (is_weighted) {
        throw std::runtime_error(
            //TODO 重み付きへの対応
            "Not implemented.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    } else {
        for (auto e : boost::make_iterator_range(boost::edges(result_network))) {
            total_length += result_network[e]->calc_length();
        }
    }

    return total_length;

}

double Simulator_WTSN::calc_total_detour(Net_2& result_network, 
                                         bool is_weighted, 
                                         std::vector<Net_2::vertex_descriptor> demand_vertices) const {
    double total_detour;

    // 移動需要点の同定
    if (demand_vertices.empty()) {
        for (auto demand_point : demand_points) {
            
            Net_2::vertex_iterator vit, vit_end;
            bool found = false;
            for (boost::tie(vit, vit_end) = boost::vertices(result_network); vit != vit_end; ++vit) {
                if(*(result_network[*vit]) == demand_point) {
                    demand_vertices.push_back(*vit);
                    found = true;
                    break;
                }
            }

            if (!found) {
                throw std::runtime_error(
                    "Could not find a demand point.\n"
                    "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                );
            }

        }
    }

    for (size_t i {0}; i < demand_vertices.size(); ++i) {
        for (size_t j {0}; j < demand_vertices.size(); ++j) {
            Net_2::vertex_descriptor Oi = demand_vertices.at(i);
            Net_2::vertex_descriptor Dj = demand_vertices.at(j);
            std::deque<std::pair<Net_2::vertex_descriptor, double>> sp_ij = result_network.calculate_shortest_path(
                Oi, 
                Dj, 
                Net_2::MODE_ROUTE, 
                true, 
                false, {}
            );
            double detour_ij = sp_ij.back().second;

            if (is_weighted) {
                total_detour += this->get_trans_prob_matrix().at(i).at(j) * detour_ij;
            } else {
                total_detour += detour_ij;
            }

        }
    }

    return total_detour;

}

std::pair<double, double> Simulator_WTSN::evaluate_network(Net_2& result_network, 
                                        bool is_weighted_length, 
                                        bool is_weighted_detour, 
                                        std::vector<Net_2::vertex_descriptor> demand_vertices, 
                                        bool checking_redundancy) const {
    double total_length {0.0};
    double total_detour {0.0};

    if (checking_redundancy) {
        // 最短路で使われないエッジはtotal_lengthに含めない
        // 移動需要点の同定
        if (demand_vertices.empty()) {
            for (auto demand_point : demand_points) {
                
                Net_2::vertex_iterator vit, vit_end;
                bool found = false;
                for (boost::tie(vit, vit_end) = boost::vertices(result_network); vit != vit_end; ++vit) {
                    if(*(result_network[*vit]) == demand_point) {
                        demand_vertices.push_back(*vit);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    throw std::runtime_error(
                        "Could not find a demand point.\n"
                        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                    );
                }

            }
        }

        std::set<Net_2::edge_descriptor> used_edges; // 最短路で使われたエッジの集合
        for (size_t i {0}; i < demand_vertices.size(); ++i) {
            for (size_t j {0}; j < demand_vertices.size(); ++j) {
                Net_2::vertex_descriptor Oi = demand_vertices.at(i);
                Net_2::vertex_descriptor Dj = demand_vertices.at(j);
                std::deque<std::pair<Net_2::vertex_descriptor, double>> sp_ij = result_network.calculate_shortest_path(
                    Oi, 
                    Dj, 
                    Net_2::MODE_ROUTE, 
                    true, 
                    false, {}
                );

                if (sp_ij.empty()) {
                    throw std::runtime_error(
                        "Could not find a shortest path from " + std::to_string(Oi) + " to " + std::to_string(Dj) + ".\n"
                        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                    );    
                }

                double detour_ij = sp_ij.back().second;

                // std::cout << "Shortest path from " << result_network[Oi]->x() << "," << result_network[Oi]->y() 
                // << " to " 
                // << result_network[Dj]->x() << "," << result_network[Dj]->y() << ": ";
                // std::cout << detour_ij << std::endl;

                if (is_weighted_detour) {
                    total_detour += this->get_trans_prob_matrix().at(i).at(j) * detour_ij;
                } else {
                    total_detour += detour_ij;
                }

                // 最短路で使われたエッジを記録
                std::pair<Net_2::edge_descriptor, bool> used_edge_existance;
                for (size_t k {0}; k < sp_ij.size() - 1; ++k) {
                    used_edge_existance = boost::edge(sp_ij.at(k).first, sp_ij.at(k + 1).first, result_network);
                    if (!used_edge_existance.second) {
                        throw std::runtime_error("Invalid adjacency (" + std::to_string(sp_ij.at(k).first) + " and " + std::to_string(sp_ij.at(k + 1).first) + ").\n"
                                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
                    } else {
                        used_edges.insert(used_edge_existance.first);
                    }
                }
            }
        }

        for (const auto& used_edge : used_edges) {
            if (is_weighted_length) {
                throw std::runtime_error(
                    //TODO 重み付きへの対応
                    "Not implemented.\n"
                    "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                );
            } else {
                total_length += result_network[used_edge]->calc_length();
            }
        }

    } else {
        total_length = calc_total_length(result_network, is_weighted_length);
        total_detour = calc_total_detour(result_network, is_weighted_detour, demand_vertices);
    }

    return std::make_pair(total_length, total_detour);

}

//** Record Functions **//
void Simulator_WTSN::save_config(std::ofstream& config_file) const {

    if (config_file.tellp() != 0) {
        throw std::runtime_error(
            "Input empty file.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    json j; // JSON形式

    // 入力: domain
    for (const auto& p : this->domain) {
        j["input"]["domain"].push_back({
            {"x", p.x()},
            {"y", p.y()}
        });
    }

    // 乱数
    j["Randim_Engine"]["seed"] = Random_Engine::get_seed();

    // pedestrian パラメータ
    j["Pedestrian"] = {
        {"mu_coef", Pedestrian::get_mu_coef()},
        {"sigma_coef", Weight_Passability_WTSN::get_pedestrian().get_sigma_coef()}
    };

    // weight パラメータ
    j["weight"] = {
        {"global_init_weight", Weight_Passability_WTSN::get_global_init_weight()},
        {"conv_weight", Weight_Passability_WTSN::get_conv_weight()},
        {"damp_count", Weight_Passability_WTSN::get_damp_count()},
        {"kappa", Weight_Passability_WTSN::get_kappa()}
    };

    // ノード
    auto node_ptrs = this->get_net_ptr()->collect_node_ptrs();
    for (const auto& node_ptr : node_ptrs) {
        j["node"]["coords"].push_back({
            {"x", node_ptr->x()},
            {"y", node_ptr->y()}
        });
    }

    // エッジ
    j["edge"]["activated_ratio"] = Edge_2_WTSN::get_activated_ratio();

    rDn_2_WTSN::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = boost::edges(*(this->get_net_ptr())); eit != eit_end; ++eit) {
        auto s = boost::source(*eit, *(this->get_net_ptr()));
        auto t = boost::target(*eit, *(this->get_net_ptr()));
        auto e_ptr = std::dynamic_pointer_cast<Edge_2_WTSN>((*(this->get_net_ptr()))[*eit]);

        j["edge"]["info"].push_back({
            {"s", s},
            {"t", t},
            {"is_fixed", e_ptr->access_weight_passability_wtsn().get_is_fixed()},
            {"default_init_weight", e_ptr->access_weight_passability_wtsn().get_default_init_weight()},
            {"amp_count_standadizer", e_ptr->access_weight_passability_wtsn().get_amp_count_standadizer()},
            {"curr_count", e_ptr->access_weight_passability_wtsn().get_curr_count()},
            {"damp_required_count", e_ptr->access_weight_passability_wtsn().get_damp_required_count()},
            {"amp_required_count", e_ptr->access_weight_passability_wtsn().get_amp_required_count()},
            {"sigmoid_band", e_ptr->access_weight_passability_wtsn().get_sigmoid_band()}
        });
    }

    // ファイルへ書き出し
    config_file << std::setw(4) << j << std::endl;

}

void Simulator_WTSN::save_log(std::ofstream& log_file) const {
    
    if (log_file.tellp() != 0) {
        throw std::runtime_error(
            "Input empty file.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // ヘッダ
    log_file << "iteration,"
             << "is_connecting," 
             << "is_converged," 
             << "unsaturated_edge_num," 
             << "activated_edge_num," 
             << "living_edge_num," 
             << "living_edge_length\n";

    // 履歴
    for (const auto& [
        iteration, 
        is_connecting, 
        is_converged, 
        unsaturated_edge_num, 
        activated_edge_num, 
        living_edge_num, 
        living_edge_length
    ] : history) 
    {
        log_file << std::scientific 
                 << std::setprecision(std::numeric_limits<double>::max_digits10) 
                 << iteration << ","
                 << is_connecting << ","
                 << is_converged << ","
                 << unsaturated_edge_num<< ","
                 << activated_edge_num << ","
                 << living_edge_num << ","
                 << living_edge_length << "\n";
    }

}