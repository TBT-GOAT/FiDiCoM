
// include standards
#include <iostream>
#include <fstream>  
#include <vector>
#include <string>

// include nlohmann-json
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// include WTSN
#include "domain/wtsn/pedestrian.h"
#include "domain/wtsn/edge_2_wtsn.h"
#include "domain/wtsn/rDn_2_wtsn.h"
#include "domain/wtsn/simulator_wtsn.h"

// include header
#include "project/TRF_SI_2025/simulation_msu.h"


//** Constructor **//
Parameters::Parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s) :    
    init_weight(init_w), 
    damp_required_count(damp_count), 
    amp_required_count(amp_count), 
    beta(b), 
    seed(s) 
{}

Parameters::Parameters(const std::vector<std::string>& args) {
    if (args.size() < 7) {
        throw std::runtime_error("Insufficient arguments for Parameters constructor.");
    }

    const std::string init_weight_str = args[2];
    const std::string damp_count_str = args[3];
    const std::string amp_count_str = args[4];
    const std::string beta_str = args[5];
    const std::string seed_str = args[6];

    this->init_weight = std::stod(init_weight_str);
    this->damp_required_count = std::stoul(damp_count_str);
    this->amp_required_count = std::stoul(amp_count_str);
    this->beta = std::stod(beta_str);
    this->seed = std::stoul(seed_str);

}

//** Simulation **//
Parameters parse_parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s) {
    return Parameters(init_w, damp_count, amp_count, b, s);
}
Parameters parse_parameters(const std::vector<std::string>& args) {
    return Parameters(args);
}

int execute_simulation_msu(double init_weight, 
                           size_t damp_required_count, 
                           size_t amp_required_count, 
                           double beta, 
                           size_t seed, 
                           const bool writing_nx) 
{
    // ランダムエンジンのシード設定
    Random_Engine::set_seed(seed);

    // ファイルの読み込み
    std::string data_folder = "/home/workspace/data/simulation_MSU/"; 
    std::string ifile_path = data_folder + "setting.json";
    std::ifstream input_file(ifile_path);
    if (!input_file) {
        std::cerr << "Error opening file: " << ifile_path << std::endl;
        return EXIT_FAILURE;
    }
    nlohmann::json input_json;
    input_file >> input_json;
    input_file.close();

    // ランダムドロネー網の初期化
    std::cout << "initialize rDn" << std::endl;
    
    std::vector<Point_2> domain_polygon;
    for (const auto& point : input_json["domain"]) {
        domain_polygon.emplace_back(point[0], point[1]);
    }

    rDn_2_WTSN rdn_2_wtsn(150000, domain_polygon);
    rdn_2_wtsn.initialize();

    // 障害物との交差判定
    std::cout << "cut edges intersecting obstacles" << std::endl;
    
    // 対象領域
    std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(rdn_2_wtsn.get_domain(), 
                                                                                            false, 
                                                                                            false, 
                                                                                            true, 
                                                                                            Obstacle_2::DOMAIN_NAME);
    rdn_2_wtsn.disconnect_edges(domain_2_ptrs);
    
    // 障害物
    std::vector<std::shared_ptr<Obstacle_2>> barrier_ptrs;
    for (const auto& barrier : input_json["barrier"]) {
        Segment_2 barrier_segment {Point_2(barrier[0], barrier[1]), 
                                    Point_2(barrier[2], barrier[3])};
        barrier_ptrs.emplace_back(std::make_shared<Obstacle_2>(barrier_segment, 
                                                                false, 
                                                                false, 
                                                                false, 
                                                                "barrier"));
    }
    rdn_2_wtsn.disconnect_edges(barrier_ptrs);

    // 建物
    std::vector<Polygon_2> building_polygons;
    for (const auto& hole : input_json["hole"]) {
        Polygon_2 hole_polygon;
        for (const auto& point : hole) {
            hole_polygon.push_back(Point_2(point[0], point[1]));
        }

        building_polygons.emplace_back(hole_polygon);

        std::vector<std::shared_ptr<Obstacle_2>> hole_ptrs = Obstacle_2::convert_polygon(hole_polygon, 
                                                                                            false, 
                                                                                            false, 
                                                                                            false, 
                                                                                            "building");
        rdn_2_wtsn.disconnect_edges(hole_ptrs);
    }

    // 移動需要点の設定
    std::cout << "set demand points" << std::endl;
    
    // 移動需要点の座標
    std::vector<Point_2> demand_points;
    for (const auto& point : input_json["demand point"]) {
        demand_points.emplace_back(point[0], point[1]);
    }

    size_t amp_count_standadizer = demand_points.size() * (demand_points.size() - 1);

    // 移動需要点の人口
    std::vector<double> demand_populations;
    for (const auto& population : input_json["demand population"]) {
        demand_populations.emplace_back(population);
    }

    // 移動需要点のタイプ
    std::vector<std::string> demand_types;
    for (const auto& type : input_json["demand type"]) {
        demand_types.emplace_back(type);
    }

    // 移動確率行列の設定
    // 移動確率は，人口×人口/距離^betaに比例
    // バス停 --> バス停の移動確率は0
    std::cout << "set moving probability matrix" << std::endl;
    

    // 距離行列をつくる
    // 距離は障害付き距離とする
    std::shared_ptr<rDn_2_WTSN> unweighted_rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
    Simulator_WTSN unweighted_simulator (rdn_2_wtsn.get_domain(), 
                                            unweighted_rdn_2_wtsn_ptr, 
                                            demand_points, 
                                            {},
                                            building_polygons);
    std::vector<rDn_2_WTSN::vertex_descriptor> unweighted_demand_nodes = unweighted_simulator.get_demand_nodes();
        
    std::vector<std::vector<double>> distance_matrix(demand_points.size(), 
                                                        std::vector<double>(demand_points.size(), 0.0)
                                                    );
    for (size_t i {0}; i < demand_points.size(); ++i) {
        for (size_t j {0}; j < demand_points.size(); ++j) {

            if (i == j) continue; // 距離は0なのでスキップ
            
            rDn_2_WTSN::vertex_descriptor source_node = unweighted_demand_nodes.at(i);
            rDn_2_WTSN::vertex_descriptor target_node = unweighted_demand_nodes.at(j);
            std::deque<std::pair<rDn_2_WTSN::vertex_descriptor, double>> shortest_path 
            = unweighted_simulator.get_net_ptr()->calculate_shortest_path(source_node, 
                                                                            target_node, 
                                                                            Net_2::MODE_ROUTE, 
                                                                            true, 
                                                                            false);
            double distance {0.0};
            if (shortest_path.empty()) {
                distance = std::numeric_limits<double>::max();
            } else {
                distance = shortest_path.back().second;
            }
            distance_matrix.at(i).at(j) = distance;
        }
    }


    // 人口×人口行列をつくる
    double total_population {0.0};
    double busstop_population {0.0};
    double entrance_population {0.0};
    double cafeteria_population {0.0};
    std::unordered_map<std::string, double> population_map;
    population_map["BUSSTOP"] = 0.0;
    population_map["RATHER"] = 0.0;
    population_map["BRYAN"] = 0.0;
    population_map["ARMSTRONG"] = 0.0;
    population_map["BAILEY"] = 0.0;
    population_map["EMMONS"] = 0.0;
    population_map["BUTTERFIELD"] = 0.0;
    population_map["BRODY"] = 0.0;
    for (size_t i {0}; i < demand_points.size(); ++i) {
        
        total_population += demand_populations.at(i);
        
        std::string type = demand_types.at(i);

        if (type == "BUSSTOP") {
            busstop_population += demand_populations.at(i);
        } else if (type == "BRODY") {
            cafeteria_population += demand_populations.at(i);
        } else {
            entrance_population += demand_populations.at(i);
        }

        if (population_map.find(type) != population_map.end()) {
            population_map[type] += demand_populations.at(i);
        } else {
            std::cerr << "Unknown demand type: " << type << std::endl;
        }
    }

    std::vector<std::vector<double>> pop_outer_matrix(demand_points.size(), 
                                                        std::vector<double>(demand_points.size(), 0.0)
                                                        );
    for (size_t i {0}; i < demand_points.size(); ++i) {
        for (size_t j {0}; j < demand_points.size(); ++j) {
            double pop_outer_value = demand_populations.at(i) * demand_populations.at(j) / total_population; // 大きすぎる値にならないようtotal_populationで割る
            pop_outer_matrix.at(i).at(j) = pop_outer_value;
        }
    }


    // 効用行列をつくる
    // 効用は人口×人口/距離^betaに比例
    std::vector<std::vector<double>> utility_matrix(demand_points.size(), 
                                                    std::vector<double>(demand_points.size(), 0.0)
                                                    );
    for (size_t i {0}; i < demand_points.size(); ++i) {
        for (size_t j {0}; j < demand_points.size(); ++j) {

            if (demand_types.at(i) == demand_types.at(j)) continue; // 同じタイプ間の効用は0

            utility_matrix.at(i).at(j) 
                = pop_outer_matrix.at(i).at(j) / std::pow(distance_matrix.at(i).at(j), beta);
        }
    }


    // 移動確率行列をつくる
    double total_prob_BUSSTOP_to_ENTRANCE {1.0/10.0};    // バス停から建物への移動確率  
    double total_prob_ENTRANCE_to_BUSSTOP {1.0/10.0};    // 建物からバス停への移動確率
    double total_prob_ENTRANCE_to_ENTRANCE {6.0/10.0};   // 建物から建物への移動確率
    double total_prob_CAFETERIA_to_ENTRANCE {1.0/10.0};  // カフェテリアから建物への移動確率
    double total_prob_ENTRANCE_to_CAFETERIA {1.0/10.0};  // 建物からカフェテリアへの移動確率

    std::vector<std::vector<double>> transition_probability_matrix(demand_points.size(), 
                                                                    std::vector<double>(demand_points.size(), 0.0)
                                                                    );
    for (size_t i {0}; i < demand_points.size(); ++i) {
        for (size_t j {0}; j < demand_points.size(); ++j) {

            if (demand_types.at(i) == demand_types.at(j)) continue; // 同じタイプ間の移動確率は0
            if (demand_types.at(i) == "BUSSTOP" && demand_types.at(j) == "BRODY") continue; // バス停からカフェテリアへの移動確率は0
            if (demand_types.at(i) == "BRODY" && demand_types.at(j) == "BUSSTOP") continue; // カフェテリアからバス停への移動確率は0

            if (demand_types.at(i) == "BUSSTOP") {
                // バス停 --> 建物
                double sum_prob 
                = total_prob_BUSSTOP_to_ENTRANCE 
                * (demand_populations.at(i) / busstop_population) 
                * (population_map[demand_types.at(j)] / entrance_population);
                
                double sum_utility {0.0};
                for (size_t k {0}; k < demand_points.size(); ++k) {
                    if (demand_types.at(k) == demand_types.at(j)) {
                        sum_utility += utility_matrix.at(i).at(k);
                    }
                }

                transition_probability_matrix.at(i).at(j)
                = sum_prob * (utility_matrix.at(i).at(j) / sum_utility);

            } else if (demand_types.at(j) == "BUSSTOP") {
                // 建物 --> バス停
                double sum_prob 
                = total_prob_ENTRANCE_to_BUSSTOP 
                * (population_map[demand_types.at(i)] / entrance_population) 
                * (demand_populations.at(j) / busstop_population);
                
                double sum_utility {0.0};
                for (size_t k {0}; k < demand_points.size(); ++k) {
                    if (demand_types.at(k) == demand_types.at(i)) {
                        sum_utility += utility_matrix.at(k).at(j);
                    }
                }

                transition_probability_matrix.at(i).at(j)
                = sum_prob * (utility_matrix.at(i).at(j) / sum_utility);

            } else if (demand_types.at(i) == "BRODY") {
                // カフェテリア --> 建物
                double sum_prob 
                = total_prob_CAFETERIA_to_ENTRANCE 
                * (population_map[demand_types.at(i)] / cafeteria_population)
                * (population_map[demand_types.at(j)] / entrance_population);

                double sum_utility {0.0};
                for (size_t k {0}; k < demand_points.size(); ++k) {
                    for (size_t l {0}; l < demand_points.size(); ++l) {
                        if (demand_types.at(k) == "BRODY" &&
                            demand_types.at(l) == demand_types.at(j)) {
                            sum_utility += utility_matrix.at(k).at(l);
                        }
                    }
                }

                transition_probability_matrix.at(i).at(j)
                = sum_prob * (utility_matrix.at(i).at(j) / sum_utility);

            } else if (demand_types.at(j) == "BRODY") {
                // 建物 --> カフェテリア
                double sum_prob
                = total_prob_ENTRANCE_to_CAFETERIA
                * (population_map[demand_types.at(i)] / entrance_population)
                * (population_map[demand_types.at(j)] / cafeteria_population);
                
                double sum_utility {0.0};
                for (size_t k {0}; k < demand_points.size(); ++k) {
                    for (size_t l {0}; l < demand_points.size(); ++l) {
                        if (demand_types.at(k) == demand_types.at(i) &&
                            demand_types.at(l) == "BRODY") {
                            sum_utility += utility_matrix.at(k).at(l);
                        }
                    }
                }

                transition_probability_matrix.at(i).at(j)
                = sum_prob * (utility_matrix.at(i).at(j) / sum_utility);

            } else {
                // 建物 --> 建物
                double sum_prob 
                = total_prob_ENTRANCE_to_ENTRANCE;
                
                double sum_utility {0.0};
                for (size_t k {0}; k < demand_points.size(); ++k) {
                    for (size_t l {0}; l < demand_points.size(); ++l) {
                        if (demand_types.at(k) != "BUSSTOP" && 
                            demand_types.at(k) != "BRODY" &&
                            demand_types.at(l) != "BUSSTOP" && 
                            demand_types.at(l) != "BRODY") {
                            sum_utility += utility_matrix.at(k).at(l);
                        }
                    }
                }

                transition_probability_matrix.at(i).at(j)
                = sum_prob * (utility_matrix.at(i).at(j) / sum_utility);

            }

        }
    }
    
    std::ofstream gf {"/home/workspace/matrix_transition_probability.txt"};
    for (const auto& tpv : transition_probability_matrix) {
        for (const double tp : tpv) {
            gf << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << tp << ",";
        }
        gf << std::endl;
    }
    
    
    // ランダムドロネー網の辺に重みを設定
    std::cout << "weight edges" << std::endl;

    // 舗装部分のポリゴン
    size_t cnt {0};
    std::vector<std::vector<Point_2>> pavement_polygons;
    for (const auto& region : input_json["region"]) {
        std::vector<Point_2> pavement_polygon;
        for (const auto& point : region) {
            pavement_polygon.push_back(Point_2(point[0], point[1]));
        }
        pavement_polygons.emplace_back(pavement_polygon);
    }

    // 舗装部分の重みの設定
    std::vector<Weight_Passability_WTSN> pavement_weights;
    for (const auto& weight: input_json["weight"]) {
        double init_weight = weight[0];
        bool is_fixed = weight[1];

        Weight_Passability_WTSN w_region_2 {init_weight, 
                                            amp_count_standadizer,  
                                            damp_required_count, 
                                            amp_required_count};
        w_region_2.set_is_fixed(is_fixed);
        pavement_weights.emplace_back(w_region_2);
    }

    // 舗装部分の領域
    std::vector<std::shared_ptr<Region_2_WTSN>> pavement_region_ptrs;
    for (size_t i = 0; i < pavement_polygons.size(); ++i) {
        const std::vector<Point_2>& pavement_polygon = pavement_polygons.at(i);
        const Weight_Passability_WTSN& pavement_weight = pavement_weights.at(i);
        Region_2_WTSN region_2_wtsn {pavement_polygon, pavement_weight};
        pavement_region_ptrs.emplace_back(std::make_shared<Region_2_WTSN>(region_2_wtsn));
    }

    // 舗装部分以外の領域
    std::shared_ptr<Region_2_WTSN> non_pavement_region_ptr 
        = std::make_shared<Region_2_WTSN>(domain_polygon, 
                                            Weight_Passability_WTSN(init_weight, 
                                                                    amp_count_standadizer, 
                                                                    damp_required_count, 
                                                                    amp_required_count)
                                            );

    // 重みの設定
    std::vector<std::shared_ptr<Region_2_WTSN>> weighted_region_ptrs;
    // 先に舗装部分（固定）を設定
    weighted_region_ptrs.insert(weighted_region_ptrs.end(), 
                                pavement_region_ptrs.begin(), 
                                pavement_region_ptrs.end());
    weighted_region_ptrs.push_back(non_pavement_region_ptr);
    // 次に草地の部分（可変）を設定
    rdn_2_wtsn.weight_edges(weighted_region_ptrs);

    
    // 実験
    std::shared_ptr<rDn_2_WTSN> rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
    Simulator_WTSN simulator (rdn_2_wtsn.get_domain(), 
                                rdn_2_wtsn_ptr, 
                                demand_points, 
                                transition_probability_matrix,
                                building_polygons);

    // セットアップ
    simulator.setup(7500, 
                    7500);
    
    // 実行
    std::cout << "execute simulation" << std::endl;
    simulator.run();
    std::cout << std::endl;

    // 移動需要点が連結できない場合は結果を空にして出力
    if (simulator.has_valid_result() == false) {
        // std::string detour_matrix_file = data_folder + "detour_matrix" + "_" + std::to_string(init_weight) + 
        //                                                                 "_" + std::to_string(damp_required_count) + 
        //                                                                 "_" + std::to_string(amp_required_count) + 
        //                                                                 "_" + std::to_string(beta) + 
        //                                                                 "_" + std::to_string(seed) + 
        //                                                                 ".csv";
        std::string detour_matrix_file = "/home/workspace/detour_matrix_" + std::to_string(init_weight) + 
                                                                "_" + std::to_string(damp_required_count) + 
                                                                "_" + std::to_string(amp_required_count) + 
                                                                "_" + std::to_string(beta) + 
                                                                "_" + std::to_string(seed) + 
                                                                ".csv";
        std::ofstream fd {detour_matrix_file};
        fd.close();

        return EXIT_FAILURE;
    }

    // 評価
    // ユークリッド距離行列を計算
    // 障害なし最短路距離で近似
    std::cout << "calculate Euclidean distance matrix" << std::endl;
    std::vector<std::vector<double>> E_distance_matrix(demand_points.size(), 
                                                       std::vector<double>(demand_points.size(), 0.0)
                                                    );
    for (size_t i {0}; i < demand_points.size(); ++i) {
        for (size_t j {0}; j < demand_points.size(); ++j) {

            if (i == j) continue; // 距離は0なのでスキップ
            
            rDn_2_WTSN::vertex_descriptor source_node = unweighted_demand_nodes.at(i);
            rDn_2_WTSN::vertex_descriptor target_node = unweighted_demand_nodes.at(j);
            std::deque<std::pair<rDn_2_WTSN::vertex_descriptor, double>> straight_path 
            = unweighted_simulator.get_net_ptr()->calculate_shortest_path(source_node, 
                                                                          target_node, 
                                                                          Net_2::MODE_ROUTE, 
                                                                          false, 
                                                                          false);
            double E_distance {0.0};
            if (straight_path.empty()) {
                E_distance = std::numeric_limits<double>::max();
            } else {
                E_distance = straight_path.back().second;
            }
            E_distance_matrix.at(i).at(j) = E_distance;
        }
    }

    // Desire Path を使った最短路距離を計算
    // WTSNと舗装路を使った最短路距離
    std::cout << "calculate Desire Path distance matrix" << std::endl;
    std::cout << "create WTSN" << std::endl;
    Net_2 wtsn(0);
    std::map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> vmap;

    std::cout << "set WTSN demand nodes" << std::endl;
    std::vector<Net_2::vertex_descriptor> wtsn_demand_nodes;
    for (const auto& demand_node : simulator.get_demand_nodes()) {
        auto v_new = boost::add_vertex((*(simulator.get_net_ptr()))[demand_node], wtsn);
        wtsn_demand_nodes.push_back(v_new);
        vmap[demand_node] = v_new;
    }

    std::cout << "set WTSN edges" << std::endl;
    rDn_2_WTSN::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = boost::edges(*(simulator.get_net_ptr())); eit != eit_end; ++eit) {
        std::shared_ptr<Edge_2_WTSN> edge_2_wtsn_ptr = std::dynamic_pointer_cast<Edge_2_WTSN>((*(simulator.get_net_ptr()))[*eit]);
        if (edge_2_wtsn_ptr->is_activated()){
            Net_2::vertex_descriptor v1_orig = boost::source(*eit, *(simulator.get_net_ptr()));
            Net_2::vertex_descriptor v2_orig = boost::target(*eit, *(simulator.get_net_ptr()));

            // v1, v2 を wtsn にコピー（未登録なら追加）
            if (vmap.count(v1_orig) == 0) {
                auto v1_new = boost::add_vertex((*(simulator.get_net_ptr()))[v1_orig], wtsn);  // Node_2をコピー
                vmap[v1_orig] = v1_new;
            }
            if (vmap.count(v2_orig) == 0) {
                auto v2_new = boost::add_vertex((*(simulator.get_net_ptr()))[v2_orig], wtsn);
                vmap[v2_orig] = v2_new;
            }
            
            std::shared_ptr<Node_2> source_ptr = (*(simulator.get_net_ptr()))[*eit]->get_source_ptr();
            std::shared_ptr<Node_2> target_ptr = (*(simulator.get_net_ptr()))[*eit]->get_target_ptr();
            Edge_2 edge(source_ptr, target_ptr);
            std::shared_ptr<Edge_2> edge_ptr = std::make_shared<Edge_2>(edge);
            boost::add_edge(vmap[v1_orig], vmap[v2_orig], edge_ptr, wtsn);
        }
    }

    std::cout << "calculate WTSN shortest path" << std::endl;
    std::vector<std::vector<double>> D_distance_matrix(wtsn_demand_nodes.size(), 
                                                       std::vector<double>(wtsn_demand_nodes.size(), 0.0));

    for (size_t i = 0; i < wtsn_demand_nodes.size(); ++i) {
        for (size_t j = 0; j < wtsn_demand_nodes.size(); ++j) {
            std::deque<std::pair<Net_2::vertex_descriptor, double>> path = wtsn.calculate_shortest_path(wtsn_demand_nodes.at(i), 
                                                                                                        wtsn_demand_nodes.at(j),
                                                                                                        Net_2::MODE_ROUTE);
            D_distance_matrix.at(i).at(j) = wtsn.calculate_path_length(path, 
                                                                       Net_2::MODE_ROUTE);
        }
    }

    // 迂回率行列を計算
    std::cout << "calculate detour matrix" << std::endl;
    std::vector<std::vector<double>> detour_matrix(demand_points.size(), 
                                                   std::vector<double>(demand_points.size(), 0.0));
    for (size_t i {0}; i < E_distance_matrix.size(); ++i) {
        for (size_t j {0}; j < E_distance_matrix.at(i).size(); ++j) {
            detour_matrix.at(i).at(j) 
                = D_distance_matrix.at(i).at(j) / E_distance_matrix.at(i).at(j);
        }
    }

    // std::string detour_matrix_file = data_folder + "detour_matrix" + "_" + std::to_string(init_weight) + 
    //                                                                  "_" + std::to_string(damp_required_count) + 
    //                                                                  "_" + std::to_string(amp_required_count) + 
    //                                                                  "_" + std::to_string(beta) + 
    //                                                                  "_" + std::to_string(seed) + 
    //                                                                  ".csv";
    std::string detour_matrix_file = "/home/workspace/detour_matrix_" + std::to_string(init_weight) + 
                                                                     "_" + std::to_string(damp_required_count) + 
                                                                     "_" + std::to_string(amp_required_count) + 
                                                                     "_" + std::to_string(beta) + 
                                                                     "_" + std::to_string(seed) + 
                                                                     ".csv";
    std::ofstream fd {detour_matrix_file};
    for (const auto& dv : detour_matrix) {
        for (const double d : dv) {
            fd << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10)
                << d << ",";
        }
        fd << std::endl;
    }
    fd.close();


    // 記録
    // 活性化している辺
    std::ofstream f {"/home/workspace/living_edges_"
                     +std::to_string(init_weight)+"_"
                     +std::to_string(damp_required_count)+"_"
                     +std::to_string(amp_required_count)+"_"
                     +std::to_string(beta)+"_"
                     +std::to_string(seed)+".txt"};
    for (auto& edge : simulator.get_living_edges()) {
            std::shared_ptr<Node_2> s_ptr = (*simulator.get_net_ptr())[edge]->get_source_ptr();
            std::shared_ptr<Node_2> t_ptr = (*simulator.get_net_ptr())[edge]->get_target_ptr();
            f << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10)
                << s_ptr->x() << " " << s_ptr->y() << " 0.0" << " "
                << t_ptr->x() << " " << t_ptr->y() << " 0.0" << std::endl;
    }

    // ログを出力
    // std::ofstream log_file("/home/workspace/log.csv");
    // simulator.save_log(log_file);

    return EXIT_SUCCESS;

}

int run_simulation_msu(const std::vector<std::string>& args, const bool writing_nx) {
    Parameters params = parse_parameters(args);
    return execute_simulation_msu(params.init_weight,
                                  params.damp_required_count,
                                  params.amp_required_count,
                                  params.beta,
                                  params.seed, 
                                  writing_nx);
}
int run_simulation_msu(double init_w, size_t damp_count, size_t amp_count, double b, size_t s, const bool writing_nx) {
    return execute_simulation_msu(init_w, damp_count, amp_count, b, s, writing_nx);
}
int run_simulation_msu(const Parameters& params, const bool writing_nx) {
    return execute_simulation_msu(params.init_weight, 
                                  params.damp_required_count, 
                                  params.amp_required_count, 
                                  params.beta, 
                                  params.seed, 
                                  writing_nx);
}