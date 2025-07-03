
// include standards
#include <iostream>
#include <chrono>
#include <filesystem>

// include pagmo
#include <pagmo/pagmo.hpp>

// include nlohmann-json
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// include CGAL library
#include <CGAL/create_offset_polygons_2.h>

// include network
#include "core/net/rDn_2.h"
#include "core/net/rDn_3.h"

// include Simukated Anealing
#include "domain/flp/SA.h"

// include FLP
#include "domain/flp/net_wp.h"
#include "domain/flp/wp_SA.h"
#include "domain/flp/net_fslp.h"
#include "domain/flp/fslp_SA.h"

// include WTSN
#include "domain/wtsn/pedestrian.h"
#include "domain/wtsn/edge_2_wtsn.h"
#include "domain/wtsn/rDn_2_wtsn.h"
#include "domain/wtsn/simulator_wtsn.h"


int main(int argc, char *argv[]) {

    int mode;
    if (argc > 1) {
        mode = std::stoi(argv[1]);
    } else {
        mode = 0;
    }

    switch (mode) {
        case 1: {
            //** 2次元 **//   
            // 対象領域の読み込み
            Polygon_2 domain_2;
        
            std::string domain_2_f_path {"/home/builder/workspace/data/domain_2.cin"};
            domain_2 = Net_2::read_domain(domain_2_f_path);
        
            // ランダムドロネー網の初期化
            std::cout << "initializing" << std::endl;
            rDn_2 rdn_2(10000, domain_2);
            rdn_2.initialize();
        
            // 障害物の読み込み
            std::string obstacle_2_f_path("/home/builder/workspace/data/obstacles_2.cin");
            std::vector<std::shared_ptr<Obstacle_2>> obstacle_2_ptrs = Obstacle_2::read_obstacles(obstacle_2_f_path);
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(domain_2, 
                                                                                            false, 
                                                                                            false, 
                                                                                            true, 
                                                                                            Obstacle_2::DOMAIN_NAME);
            obstacle_2_ptrs.insert(obstacle_2_ptrs.end(), domain_2_ptrs.begin(), domain_2_ptrs.end());
        
            // 障害物との交差判定
            std::cout << "disconnecting" << std::endl;
            rdn_2.disconnect_edges(obstacle_2_ptrs);
        
            // ネットワークの書き出し
            std::string node_2_f_path {"/home/builder/workspace/data/nodes_2.cout"};
            std::string edge_2_f_path {"/home/builder/workspace/data/edges_2.cout"};
            std::string adjacency_2_f_path {"/home/builder/workspace/data/adjacency_2.cout"};
            std::string cell_2_f_path {"/home/builder/workspace/data/cells_2.cout"};
        
            rdn_2.write_nodes(node_2_f_path);
            rdn_2.write_edges(edge_2_f_path);
            rdn_2.write_adjacency(adjacency_2_f_path, Net_2::MODE_VISIBILITY);
            rdn_2.write_cells(cell_2_f_path);
        
            // 最短経路木の書き出し
            std::cout << "calculating shortest path tree" << std::endl;
            Point_2 p_2 (0.0, 0.0);
            Net_2::vertex_descriptor vantage_node_2 = rdn_2.find_nearest_node(p_2);
        
            std::vector<std::pair<Net_2::vertex_descriptor, double>> spt_v_2 = rdn_2.calculate_shortest_path_tree(vantage_node_2, Net_2::MODE_VISIBILITY, true, true);
            
            std::ofstream f_2("/home/builder/workspace/data/spt_visibility_2.cout");
            for (size_t i {0}; i < spt_v_2.size(); ++i) {
                f_2 << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << i << " " << spt_v_2.at(i).first << " " << spt_v_2.at(i).second << std::endl;
            }
        
            std::vector<std::pair<Net_2::vertex_descriptor, double>> spt_r_2 = rdn_2.calculate_shortest_path_tree(vantage_node_2, Net_2::MODE_ROUTE, true, true);
            
            std::ofstream g_2("/home/builder/workspace/data/spt_route_2.cout");
            for (size_t i {0}; i < spt_r_2.size(); ++i) {
                g_2 << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << i << " " << spt_r_2.at(i).first << " " << spt_r_2.at(i).second << std::endl;
            }
        
            // 可視ノードの書き出し
            std::cout << "calculating visibility" << std::endl;
            std::unordered_set<Net_2::vertex_descriptor> visible_vertices_2 = rdn_2.calculate_visible_vertices(p_2, 500.0);
        
            std::ofstream h_2("/home/builder/workspace/data/visible_nodes_2.cout");
            for (const auto& v_2 : visible_vertices_2) {
                h_2 << v_2 << " ";
            }
        
            // ビジョンの書き出し
            std::cout << "calculating panoramic_vision" << std::endl;
            std::unordered_map<std::string, std::vector<std::pair<Point_2, Point_2>>> panoramic_vision_2;
            panoramic_vision_2 = rdn_2.calculate_panoramic_vision(p_2);
            
            std::ofstream j_2("/home/builder/workspace/data/panoramic_vision_2.cout");
            for (const auto& entry : panoramic_vision_2) {
                j_2 << "#" << entry.first << std::endl;
                for (const auto& cell : entry.second) {
                    j_2 << cell.first.x() << "," << cell.first.y() << " ";
                    j_2 << cell.second.x() << "," << cell.second.y() << " ";
                    j_2 << std::endl;
                }
                j_2 << std::endl;
            }
        
            // 到達可能なノードの書き出し
            std::cout << "calculating reachability" << std::endl;
            std::unordered_set<Net_2::vertex_descriptor> reachable_vertices_2 = rdn_2.calculate_reachable_vertices(p_2, 1000.0);
        
            std::ofstream i_2("/home/builder/workspace/data/reachable_nodes_2.cout");
            for (const auto& v : reachable_vertices_2) {
                i_2 << v << " ";
            }

            break;
        }
        case 2: {
            //** 3次元 **//
            std::string base_path = std::filesystem::canonical(".").string() + "/data/";

            // 対象領域の読み込み
            Polyhedron_3 domain_3;

            std::string domain_3_f_path {base_path+"domain_3.stl"};
            domain_3 = Net_3::read_domain(domain_3_f_path);

            // ランダムドロネー網の初期化
            std::cout << "initializing" << std::endl;
            rDn_3 rdn_3(100000, domain_3);
            rdn_3.initialize();

            // 障害物の読み込み
            // std::string obstacle_3_f_path(base_path+"obstacles_3.stl");
            // std::vector<std::shared_ptr<Obstacle_3>> obstacle_3_ptrs = Obstacle_3::read_obstacles(obstacle_3_f_path, 
            //                                                                                     false, 
            //                                                                                     false, 
            //                                                                                     false);
            
            std::vector<std::shared_ptr<Obstacle_3>> obstacle_3_ptrs;
            for (size_t i {1}; i < 37; ++i) {
                std::string obstacle_3_f_path(base_path + "o" + std::to_string(i) + ".stl");
                std::vector<std::shared_ptr<Obstacle_3>> obstacle_3_ptrs_tmp = Obstacle_3::read_obstacles(obstacle_3_f_path, 
                                                                                                        false, 
                                                                                                        false, 
                                                                                                        false);
                obstacle_3_ptrs.insert(obstacle_3_ptrs.end(), obstacle_3_ptrs_tmp.begin(), obstacle_3_ptrs_tmp.end());
            }

            
            std::vector<std::shared_ptr<Obstacle_3>> domain_3_ptrs = Obstacle_3::convert_polyhedron(domain_3, 
                                                                                                false, 
                                                                                                false, 
                                                                                                true, 
                                                                                                Obstacle_3::DOMAIN_NAME);

            obstacle_3_ptrs.insert(obstacle_3_ptrs.end(), domain_3_ptrs.begin(), domain_3_ptrs.end());

            // 障害物との交差判定
            std::cout << "disconnecting" << std::endl;
            rdn_3.disconnect_edges(obstacle_3_ptrs);    

            // ネットワークの書き出し
            std::string node_3_f_path {base_path+"nodes_3.cout"};
            std::string edge_3_f_path {base_path+"edges_3.cout"};
            std::string adjacency_3_f_path {base_path+"adjacency_3.cout"};

            rdn_3.write_nodes(node_3_f_path);
            rdn_3.write_edges(edge_3_f_path);
            rdn_3.write_adjacency(adjacency_3_f_path, Net_3::MODE_VISIBILITY);

            // 最短経路木の書き出し
            std::cout << "calculating shortest path tree" << std::endl;
            // Point_3 p_3 (7000, 2000, -2000);
            Point_3 p_3 (50000, 50000, 25000);
            Net_3::vertex_descriptor vantage_node_3 = rdn_3.find_nearest_node(p_3);

            std::vector<std::pair<Net_3::vertex_descriptor, double>> spt_v_3 = rdn_3.calculate_shortest_path_tree(vantage_node_3, Net_3::MODE_VISIBILITY, true, true);
            
            std::ofstream f_3(base_path+"spt_visibility_3.cout");
            for (size_t i {0}; i < spt_v_3.size(); ++i) {
                f_3 << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << i << " " << spt_v_3.at(i).first << " " << spt_v_3.at(i).second << std::endl;
            }

            // 可視ノードの書き出し
            std::cout << "calculating visibility" << std::endl;
            std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
            std::unordered_set<Net_3::vertex_descriptor> visible_vertices_3 = rdn_3.calculate_visible_vertices(p_3);//, 10000.0);
            std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << duration << " milisec" << std::endl;
            
            std::vector<std::shared_ptr<Node_3>> visible_node_3_ptrs;

            std::ofstream h_3(base_path+"visible_nodes_3.cout");
            for (const auto& v : visible_vertices_3) {
                h_3 << v << " ";
                visible_node_3_ptrs.push_back(rdn_3[v]);
            }

            std::string cell_3_f_path {base_path+"visible_cells_3.stl"};
            rdn_3.write_cells(cell_3_f_path, visible_node_3_ptrs);

            // ビジョンの書き出し
            std::cout << "calculating panoramic_vision" << std::endl;
            std::unordered_map<std::string, std::vector<std::vector<Point_on_sphere_3>>> panoramic_vision_3;
            panoramic_vision_3 = rdn_3.calculate_panoramic_vision(p_3, std::numeric_limits<double>::max(), 100000.0);
            
            std::ofstream j_3(base_path+"panoramic_vision_3.cout");
            for (const auto& entry : panoramic_vision_3) {
                j_3 << "#" << entry.first << std::endl;
                for (const auto& cell : entry.second) {
                    for (const auto& point : cell) {
                        j_3 << point.x() << "," << point.y() << "," << point.z() << " ";
                    }
                    j_3 << std::endl;
                }
                j_3 << std::endl;
            }

            // 到達可能なノードの書き出し
            std::cout << "calculating reachability" << std::endl;
            std::unordered_set<Net_3::vertex_descriptor> reachable_vertices_3 = rdn_3.calculate_reachable_vertices(p_3, 10000.0);

            std::ofstream i_3(base_path+"reachable_nodes_3.cout");
            for (const auto& v : reachable_vertices_3) {
                i_3 << v << " ";
            }

            break;
        }
        case 3: {
            //** Weber Problem のテスト **//
            Random_Engine::set_seed(25);

            // 対象領域の読み込み
            Polygon_2 domain_2_wp;

            std::string domain_2_wp_f_path {"/home/builder/workspace/data/domain_2_fslp.cin"};
            domain_2_wp = Net_2::read_domain(domain_2_wp_f_path);

            // ランダムドロネー網の初期化
            std::cout << "initializing" << std::endl;
            rDn_2 rdn_2_wp(10000, domain_2_wp);
            rdn_2_wp.initialize();

            // 障害物の読み込み
            std::string obstacle_2_wp_f_path("/home/builder/workspace/data/obstacles_2_fslp.cin");
            std::vector<std::shared_ptr<Obstacle_2>> obstacle_2_wp_ptrs = Obstacle_2::read_obstacles(obstacle_2_wp_f_path);
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_wp_ptrs = Obstacle_2::convert_polygon(domain_2_wp, 
                                                                                            false, 
                                                                                            false, 
                                                                                            true, 
                                                                                            Obstacle_2::DOMAIN_NAME);
            obstacle_2_wp_ptrs.insert(obstacle_2_wp_ptrs.end(), domain_2_wp_ptrs.begin(), domain_2_wp_ptrs.end());

            // 障害物との交差判定
            std::cout << "disconnecting" << std::endl;
            rdn_2_wp.disconnect_edges(obstacle_2_wp_ptrs);

            // ソルバの設定
            typedef std::vector<Net_2::vertex_descriptor> Facilities;

            std::shared_ptr<Net_2> rdn_2_wp_ptr = std::make_shared<rDn_2>(rdn_2_wp);
            Net_WP net_wp(rdn_2_wp_ptr);
            Point_2 inner_point(0.0, 0.0);
            size_t facility_num {16};
            size_t sign_num {0};
            std::vector<Point_2> main_buildings {Point_2(0.0, 0.0)};
            net_wp.set_demands(inner_point);
            net_wp.initialize_facilities(facility_num);
        
            Facilities initial_solution = net_wp.get_facilities();

            std::shared_ptr<WP_SA> solver_ptr = std::make_shared<WP_SA>(net_wp);
            size_t mode = WP_SA::MODE_MINSUM;
            bool show_progress = true;

            Simulated_Annealing<Facilities> sa(
                1000.0,                              // 初期温度
                0.999,                              // 冷却率
                5000,                              // 最大反復回数
                [solver_ptr, mode](const Facilities& solution){
                    return solver_ptr->evaluate_function(solution, mode);
                }, 
                [solver_ptr](const Facilities& current_solution){
                    return solver_ptr->generate_neighbor_function(current_solution);
                }
            );

            // 求解
            std::cout << "Initial" << std::endl;
            std::cout << "Facility" << std::endl;
            for (const auto& facility : initial_solution) {
                Node_2 facility_node = *((*(solver_ptr->net_wp.net_ptr))[facility]);
                std::cout << facility_node.x() << "," << facility_node.y() << ",0.0" << std::endl;
            }

            Facilities best_solution = sa.solve(initial_solution, show_progress);

            std::cout << "WP Solved" << std::endl;
            std::cout << "Facility" << std::endl;
            for (const auto& facility : best_solution) {
                Node_2 facility_node = *((*(solver_ptr->net_wp.net_ptr))[facility]);
                std::cout << facility_node.x() << "," << facility_node.y() << ",0.0" << std::endl;
            }

            break;
        }
        case 4: {
            //** Facility-Sign Location Problem のテスト1 **//
            Random_Engine::set_seed(25);

            // 対象領域の読み込み
            Polygon_2 domain_2_fslp;

            std::string domain_2_fslp_f_path {"/home/builder/workspace/data/domain_2_fslp.cin"};
            domain_2_fslp = Net_2::read_domain(domain_2_fslp_f_path);

            // ランダムドロネー網の初期化
            std::cout << "initializing" << std::endl;
            rDn_2 rdn_2_fslp(1000, domain_2_fslp);
            rdn_2_fslp.initialize();

            // 障害物の読み込み
            std::string obstacle_2_fslp_f_path("/home/builder/workspace/data/obstacles_2_fslp.cin");
            std::vector<std::shared_ptr<Obstacle_2>> obstacle_2_fslp_ptrs = Obstacle_2::read_obstacles(obstacle_2_fslp_f_path);
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_fslp_ptrs = Obstacle_2::convert_polygon(domain_2_fslp, 
                                                                                            false, 
                                                                                            false, 
                                                                                            true, 
                                                                                            Obstacle_2::DOMAIN_NAME);
            obstacle_2_fslp_ptrs.insert(obstacle_2_fslp_ptrs.end(), domain_2_fslp_ptrs.begin(), domain_2_fslp_ptrs.end());

            // 障害物との交差判定
            std::cout << "disconnecting" << std::endl;
            rdn_2_fslp.disconnect_edges(obstacle_2_fslp_ptrs);

            // ソルバの設定
            typedef std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> Facilities_Signs_Pair;
            
            std::shared_ptr<Net_2> rdn_2_fslp_ptr = std::make_shared<rDn_2>(rdn_2_fslp);
            Net_FSLP net_fslp(rdn_2_fslp_ptr);
            net_fslp.set_visible_length(std::sqrt(2)*10000/3);
            Point_2 inner_point(25000.000,25000.000);
            std::vector<Point_2> facility_points {Point_2(8333.333,8333.333), 
                                                Point_2(41666.667,8333.333), 
                                                Point_2(25000.000,25000), 
                                                Point_2(8333.333,41666.667), 
                                                Point_2(41666.667,41666.667)};
            std::vector<Point_2> sign_points {Point_2(25000.000,8333.333), 
                                                Point_2(8333.333,25000.000), 
                                                Point_2(41666.667,25000.000), 
                                                Point_2(25000.000,41666.667)};
            std::vector<Point_2> main_buildings {Point_2(2500.000,8333.333,0.000)};

            net_fslp.set_demands(inner_point);

            std::ofstream file0("/home/builder/workspace/data/demands_fslp.cout");
            std::cout << "Demands" << std::endl;
            for (const auto& demand : net_fslp.get_demands()) {
                file0 << (*(net_fslp.net_ptr))[demand]->x() << "," << (*(net_fslp.net_ptr))[demand]->y() << "," << 0.0 << std::endl;
            }

            net_fslp.initialize_facilities(facility_points);
            net_fslp.initialize_signs(sign_points);
            net_fslp.initialize_main_buildings(main_buildings);

            net_fslp.build_trees();
            net_fslp.build_assignments();


            std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_demand = net_fslp.get_facility_assignment_to_demand();
            
            std::ofstream file1a("/home/builder/workspace/data/facility_assignment_to_demand.cout");
            for (const auto& [demand, facility] : facility_assignment_to_demand) {
                if (demand == facility) continue;

                file1a << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << "line "
                << (*(net_fslp.net_ptr))[demand]->x() << "," << (*(net_fslp.net_ptr))[demand]->y() << ",0.0" << " "
                << (*(net_fslp.net_ptr))[facility]->x() << "," << (*(net_fslp.net_ptr))[facility]->y() << ",0.0" << " " << std::endl;
            }

            std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> sign_assignment_to_demand = net_fslp.get_sign_assignment_to_demand();
            
            std::ofstream file1b("/home/builder/workspace/data/sign_assignment_to_demand.cout");
            for (const auto& [demand, sign] : sign_assignment_to_demand) {
                if (demand == sign) continue;

                file1b << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << "line "
                << (*(net_fslp.net_ptr))[demand]->x() << "," << (*(net_fslp.net_ptr))[demand]->y() << ",0.0" << " "
                << (*(net_fslp.net_ptr))[sign]->x() << "," << (*(net_fslp.net_ptr))[sign]->y() << ",0.0" << " " << std::endl;
            }

            std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_sign = net_fslp.get_facility_assignment_to_sign();
            
            std::ofstream file1c("/home/builder/workspace/data/facility_assignment_to_sign.cout");
            for (const auto& [facility, sign] : facility_assignment_to_sign) {
                if (facility == sign) continue;

                file1c << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << "line "
                << (*(net_fslp.net_ptr))[facility]->x() << "," << (*(net_fslp.net_ptr))[facility]->y() << ",0.0" << " "
                << (*(net_fslp.net_ptr))[sign]->x() << "," << (*(net_fslp.net_ptr))[sign]->y() << ",0.0" << " " << std::endl;
            }

            std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_main_building = net_fslp.get_facility_assignment_to_main_building();
            
            std::ofstream file1d("/home/builder/workspace/data/facility_assignment_to_main_building.cout");
            for (const auto& [facility, main_building] : facility_assignment_to_main_building) {
                if (facility == main_building) continue;

                file1d << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << "line "
                << (*(net_fslp.net_ptr))[facility]->x() << "," << (*(net_fslp.net_ptr))[facility]->y() << ",0.0" << " "
                << (*(net_fslp.net_ptr))[main_building]->x() << "," << (*(net_fslp.net_ptr))[main_building]->y() << ",0.0" << " " << std::endl;
            }

            std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> main_building_assignment_to_demand = net_fslp.get_main_building_assignment_to_demand();
            
            std::ofstream file1e("/home/builder/workspace/data/main_building_assignment_to_demand.cout");
            for (const auto& [main_building, demand] : main_building_assignment_to_demand) {
                if (main_building == demand) continue;

                file1e << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << "line "
                << (*(net_fslp.net_ptr))[main_building]->x() << "," << (*(net_fslp.net_ptr))[main_building]->y() << ",0.0" << " "
                << (*(net_fslp.net_ptr))[demand]->x() << "," << (*(net_fslp.net_ptr))[demand]->y() << ",0.0" << " " << std::endl;
            }


            std::string node_2_fslp_f_path {"/home/builder/workspace/data/nodes_2_fslp.cout"};
            net_fslp.net_ptr->write_nodes(node_2_fslp_f_path);


            std::unordered_map<Net_2::vertex_descriptor, 
                               std::vector<std::pair<Net_2::vertex_descriptor, double>>
            > facility_coverage_trees = net_fslp.get_facility_coverage_trees();
            for (const auto& [facility, coverage_tree] : facility_coverage_trees) {
                std::ofstream file1("/home/builder/workspace/data/facility_coverage_tree_" + std::to_string(facility) + ".cout");
                for (size_t i {0}; i < coverage_tree.size(); ++i) {
                    file1 << std::scientific 
                    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                    << i << " " << coverage_tree.at(i).first << " " << coverage_tree.at(i).second << std::endl;
                }
            }


            std::vector<std::pair<Net_2::vertex_descriptor, double>> facility_shortest_path_tree = net_fslp.get_facility_shortest_path_tree();
            
            std::ofstream file2("/home/builder/workspace/data/facility_shortest_path_tree.cout");
            for (size_t i {0}; i < facility_shortest_path_tree.size(); ++i) {
                file2 << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << i << " " << facility_shortest_path_tree.at(i).first << " " << facility_shortest_path_tree.at(i).second << std::endl;
            }


            std::unordered_map<Net_2::vertex_descriptor, 
                               std::vector<std::pair<Net_2::vertex_descriptor, double>>
            > sign_coverage_trees = net_fslp.get_sign_coverage_trees();
            for (const auto& [sign, coverage_tree] : sign_coverage_trees) {
                std::ofstream file3("/home/builder/workspace/data/sign_coverage_tree_" + std::to_string(sign) + ".cout");
                for (size_t i {0}; i < coverage_tree.size(); ++i) {
                    file3 << std::scientific 
                    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                    << i << " " << coverage_tree.at(i).first << " " << coverage_tree.at(i).second << std::endl;
                }
            }


            std::vector<std::pair<Net_2::vertex_descriptor, double>> main_building_shortest_path_tree = net_fslp.get_main_building_shortest_path_tree();
            
            std::ofstream file4("/home/builder/workspace/data/main_building_shortest_path_tree.cout");
            for (size_t i {0}; i < main_building_shortest_path_tree.size(); ++i) {
                file4 << std::scientific 
                << std::setprecision(std::numeric_limits<double>::max_digits10) 
                << i << " " << main_building_shortest_path_tree.at(i).first << " " << main_building_shortest_path_tree.at(i).second << std::endl;
            }


            std::vector<Point_2> test_demands {Point_2(5000.000,8333.333), 
                                               Point_2(45000.000,8333.333), 
                                               Point_2(25000.000,16666.667), 
                                               Point_2(8333.333,38333.333), 
                                               Point_2(41666.667,38333.333)};

            for (const auto& test_demand : test_demands) {
                std::cout << "\ntest " << test_demand << std::endl;
                Net_2::vertex_descriptor test_demand_vertex = net_fslp.search_nearest_demand(test_demand);
                std::cout << "test demand position " << (*(net_fslp.net_ptr))[test_demand_vertex]->x() << "," << (*(net_fslp.net_ptr))[test_demand_vertex]->y() << ",0.0" << std::endl;
                std::cout << net_fslp.calculate_cost(test_demand_vertex) << std::endl;
            }

            break;
        }
        case 5: {
            //** Facility-Sign Location Problem のテスト2 **//
            Random_Engine::set_seed(17);

            // 対象領域の読み込み
            Polygon_2 domain_2_fslp;

            std::string domain_2_fslp_f_path {"/home/builder/workspace/data/domain_2_fslp.cin"};
            domain_2_fslp = Net_2::read_domain(domain_2_fslp_f_path);

            // ランダムドロネー網の初期化
            std::cout << "initializing" << std::endl;
            rDn_2 rdn_2_fslp(10000, domain_2_fslp);
            rdn_2_fslp.initialize();

            // 障害物の読み込み
            std::string obstacle_2_fslp_f_path("/home/builder/workspace/data/obstacles_2_fslp.cin");
            std::vector<std::shared_ptr<Obstacle_2>> obstacle_2_fslp_ptrs = Obstacle_2::read_obstacles(obstacle_2_fslp_f_path);
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_fslp_ptrs = Obstacle_2::convert_polygon(domain_2_fslp, 
                                                                                                false, 
                                                                                                false, 
                                                                                                true, 
                                                                                                Obstacle_2::DOMAIN_NAME);
            obstacle_2_fslp_ptrs.insert(obstacle_2_fslp_ptrs.end(), domain_2_fslp_ptrs.begin(), domain_2_fslp_ptrs.end());

            // 障害物との交差判定
            std::cout << "disconnecting" << std::endl;
            rdn_2_fslp.disconnect_edges(obstacle_2_fslp_ptrs);

            // ソルバの設定
            typedef std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> Facilities_Signs_Pair;
            
            std::shared_ptr<Net_2> rdn_2_fslp_ptr = std::make_shared<rDn_2>(rdn_2_fslp);
            Net_FSLP net_fslp(rdn_2_fslp_ptr);
            Point_2 inner_point(25000.0, 25000.0);
            size_t facility_num {9};
            size_t sign_num {12};
            std::vector<Point_2> main_buildings {Point_2(25000.0, 25000.0)};
            double visible_length = std::sqrt(2)*10000/3;
            
            net_fslp.set_visible_length(visible_length);
            net_fslp.set_demands(inner_point);
            net_fslp.initialize_facilities(facility_num);
            net_fslp.initialize_signs(sign_num);
            net_fslp.initialize_main_buildings(main_buildings);

            Facilities_Signs_Pair initial_solution = std::make_pair(net_fslp.get_facilities(), net_fslp.get_signs());

            std::shared_ptr<FSLP_SA> solver_ptr = std::make_shared<FSLP_SA>(net_fslp);
            size_t mode = FSLP_SA::MODE_MINSUM;
            bool show_progress = true;
            bool logging = true;
            std::ofstream log_file("/home/builder/workspace/data/log_fslp.cout");

            Simulated_Annealing<Facilities_Signs_Pair> sa(
                1000.0,     // 初期温度
                0.999,      // 冷却率
                2000,       // 最大反復回数
                [solver_ptr, mode](const Facilities_Signs_Pair solution){
                    return solver_ptr->evaluate_function(solution, mode);
                }, 
                [solver_ptr](const Facilities_Signs_Pair& current_solution){
                    return solver_ptr->generate_neighbor_function_with_jump(current_solution);
                }
            );

            // 求解
            std::cout << "Initial" << std::endl;
            std::cout << "Facility" << std::endl;
            for (const auto& facility : initial_solution.first) {
                Node_2 facility_node = *((*(solver_ptr->net_fslp.net_ptr))[facility]);
                std::cout << facility_node.x() << "," << facility_node.y() << ",0.0" << std::endl;
            }
            std::cout << "Sign" << std::endl;
            for (const auto& sign : initial_solution.second) {
                Node_2 sign_node = *((*(solver_ptr->net_fslp.net_ptr))[sign]);
                std::cout << sign_node.x() << "," << sign_node.y() << ",0.0" << std::endl;
            }
            std::cout << "Main Building" << std::endl;
            for (const auto& main_building : solver_ptr->net_fslp.get_main_buildings()) {
                Node_2 main_building_node = *((*(solver_ptr->net_fslp.net_ptr))[main_building]);
                std::cout << main_building_node.x() << "," << main_building_node.y() << ",0.0" << std::endl;
            }

            Facilities_Signs_Pair best_solution = sa.solve(initial_solution, show_progress, logging, &log_file);

            std::cout << "FSLP Solved" << std::endl;
            std::cout << "Facility" << std::endl;
            for (const auto& facility : best_solution.first) {
                Node_2 facility_node = *((*(solver_ptr->net_fslp.net_ptr))[facility]);
                std::cout << facility_node.x() << "," << facility_node.y() << ",0.0" << std::endl;
            }
            std::cout << "Sign" << std::endl;
            for (const auto& sign : best_solution.second) {
                Node_2 sign_node = *((*(solver_ptr->net_fslp.net_ptr))[sign]);
                std::cout << sign_node.x() << "," << sign_node.y() << ",0.0" << std::endl;
            }

            break;
        }
        case 6 :{
            struct OptimizationProblem {
                pagmo::vector_double fitness(const pagmo::vector_double &x) const {
                    double result = 0.0;
                    for (double xi : x) result += xi * xi; // ダミーの目的関数
                    return {result};
                }
            
                std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const {
                    return {{0, 0, 0}, {1, 1, 1}}; // 例: 0～1 の範囲で選択
                }
            };

            std::cout << "pagmo" << std::endl;
            
            pagmo::problem prob{OptimizationProblem()};
            pagmo::algorithm algo{pagmo::pso(100)};  // 100個の粒子で最適化
            pagmo::population pop{prob, 50};  // 50個の候補解で開始
        
            pop = algo.evolve(pop);  // 最適化実行
        
            std::cout << "Best solution: " << pop.champion_x()[0] << "\n";
            
            break;
        }
        case 7: {
            //** Facility-Sign Location Problem の分析 **//
            std::string base_path = std::filesystem::canonical(".").string() + "/data/";
            typedef std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> Facilities_Signs_Pair;
            
            // 分析条件
            size_t trial_num {10};
            std::vector<size_t> facility_nums(9);
            std::iota(facility_nums.begin(), facility_nums.end(), 1);
            std::vector<size_t> sign_nums(10);
            std::iota(sign_nums.begin(), sign_nums.end(), 0);

            Point_2 inner_point(8333.333, 8333.333);
            std::vector<Point_2> main_buildings {inner_point};
            double visible_length = std::sqrt(2)*10000/3;

            double init_temperature = 1000.0;     // 初期温度
            double cooling_rate = 0.999;          // 冷却率
            double max_iter = 2000;               // 最大反復回数
            
            size_t mode = FSLP_SA::MODE_MINSUM;
            bool show_progress = false;
            bool logging = true;

            
            // 対象領域の読み込み
            Polygon_2 domain_2_fslp;

            std::string domain_2_fslp_f_path {base_path+"domain_2_fslp.cin"};
            domain_2_fslp = Net_2::read_domain(domain_2_fslp_f_path);
            
            // 障害物の読み込み
            std::string obstacle_2_fslp_f_path(base_path+"obstacles_2_fslp.cin");
            std::vector<std::shared_ptr<Obstacle_2>> obstacle_2_fslp_ptrs = Obstacle_2::read_obstacles(obstacle_2_fslp_f_path);
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_fslp_ptrs = Obstacle_2::convert_polygon(domain_2_fslp, 
                                                                                                false, 
                                                                                                false, 
                                                                                                true, 
                                                                                                Obstacle_2::DOMAIN_NAME);
            obstacle_2_fslp_ptrs.insert(obstacle_2_fslp_ptrs.end(), domain_2_fslp_ptrs.begin(), domain_2_fslp_ptrs.end());

            // ランダムドロネー網の初期化
            rDn_2 rdn_2_fslp(10000, domain_2_fslp);
            rdn_2_fslp.initialize();

            // 障害物との交差判定
            rdn_2_fslp.disconnect_edges(obstacle_2_fslp_ptrs);

            std::shared_ptr<Net_2> rdn_2_fslp_ptr = std::make_shared<rDn_2>(rdn_2_fslp);


            // 分析
            for (const auto& facility_num : facility_nums) {
                for (const auto& sign_num : sign_nums) {

                    std::ofstream result_file(base_path+"result_fslp_" + std::to_string(mode) + "_"  + std::to_string(facility_num) + "_" + std::to_string(sign_num) + ".cout");

                    for (size_t trial {0}; trial < trial_num; trial++) {

                        Random_Engine::set_seed(trial);

                        // ソルバの設定                        
                        Net_FSLP net_fslp(rdn_2_fslp_ptr);
                        net_fslp.set_visible_length(visible_length);
                        net_fslp.set_demands(inner_point);
                        net_fslp.initialize_facilities(facility_num);
                        net_fslp.initialize_signs(sign_num);
                        net_fslp.initialize_main_buildings(main_buildings);

                        Facilities_Signs_Pair initial_solution = std::make_pair(net_fslp.get_facilities(), net_fslp.get_signs());

                        std::shared_ptr<FSLP_SA> solver_ptr = std::make_shared<FSLP_SA>(net_fslp);
                        std::string log_file_name = base_path+"log_fslp_" + std::to_string(mode) + "_" + std::to_string(trial) + "_" + std::to_string(facility_num) + "_" + std::to_string(sign_num) + ".cout";
                        std::ofstream log_file(log_file_name);

                        Simulated_Annealing<Facilities_Signs_Pair> sa(
                            init_temperature,    
                            cooling_rate,      
                            max_iter,       
                            [solver_ptr, mode](const Facilities_Signs_Pair solution){
                                return solver_ptr->evaluate_function(solution, mode);
                            }, 
                            [solver_ptr](const Facilities_Signs_Pair& current_solution){
                                return solver_ptr->generate_neighbor_function_with_jump(current_solution);
                            }
                        );

                        // 求解
                        Facilities_Signs_Pair best_solution = sa.solve(initial_solution, show_progress, logging, &log_file);

                        // 結果の記録
                        result_file << std::scientific 
                        << std::setprecision(std::numeric_limits<double>::max_digits10) 
                        << "trial: " << trial << std::endl
                        << "cost: " << solver_ptr->evaluate_function(best_solution, mode) << std::endl
                        << "facilities: " << std::endl;
                        for (const auto& facility : best_solution.first) {
                            Node_2 facility_node = *((*(solver_ptr->net_fslp.net_ptr))[facility]);
                            result_file << std::scientific 
                            << std::setprecision(std::numeric_limits<double>::max_digits10) 
                            << facility_node.x() << "," << facility_node.y() << ",0.0" << std::endl;
                        }
                        result_file << "signs: " << std::endl;
                        for (const auto& sign : best_solution.second) {
                            Node_2 sign_node = *((*(solver_ptr->net_fslp.net_ptr))[sign]);
                            result_file << std::scientific 
                            << std::setprecision(std::numeric_limits<double>::max_digits10) 
                            << sign_node.x() << "," << sign_node.y() << ",0.0" << std::endl;
                        }
                        result_file << "main building: " << std::endl;
                        for (const auto& main_building : solver_ptr->net_fslp.get_main_buildings()) {
                            Node_2 main_building_node = *((*(solver_ptr->net_fslp.net_ptr))[main_building]);
                            result_file << std::scientific 
                            << std::setprecision(std::numeric_limits<double>::max_digits10) 
                            << main_building_node.x() << "," << main_building_node.y() << ",0.0" << std::endl;
                        }
                        
                        result_file << std::endl;
                        
                        // 設定をもとに戻す
                        net_fslp.clear();

                    }
                }

            }

            break;

        }
        case 8: {
            //** Pedestrian のテスト **//
            const int NUM_SAMPLES = 1000000;  // サンプル数を多めに（100万回）
            std::vector<double> history (NUM_SAMPLES);

            try {
                Pedestrian pedestrian(-0.5);  // mu_coef = -1 → 平均1になるはず

                for (int i = 0; i < NUM_SAMPLES; ++i) {
                    pedestrian.change_pedestrian();
                    double coef = pedestrian.get_coef();
                    history.at(i) = coef;
                }

                std::sort(history.begin(), history.end());
                double empirical_median;
                if (NUM_SAMPLES % 2 == 0) {
                    empirical_median = (history[NUM_SAMPLES / 2 - 1] + history[NUM_SAMPLES / 2]) / 2.0;
                } else {
                    empirical_median = history[NUM_SAMPLES / 2];
                }
                
                std::cout << "Sample size       : " << NUM_SAMPLES << std::endl;
                std::cout << "Empirical median  : " << empirical_median << std::endl;
                std::cout << "Expected median     : 1.0" << std::endl;
        
            } catch (const std::exception& ex) {
                std::cerr << "Exception: " << ex.what() << std::endl;
            }
        }
        case 9: {
            //** 重み付き最短路のテスト **//

            // 乱数の種の設定
            Random_Engine::set_seed(17);

            // ランダムドロネー網の初期化
            std::cout << "initialize rDn" << std::endl;
            std::vector<Point_2> domain_2_vertices {{0.0, 0.0}, 
                                                    {180.0, 0.0}, 
                                                    {180.0, 105.0}, 
                                                    {0.0, 105.0}};
            Polygon_2 domain_2 = Polygon_2(domain_2_vertices.begin(), domain_2_vertices.end());
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(domain_2, 
                                                                                                 false, 
                                                                                                 false, 
                                                                                                 true, 
                                                                                                 Obstacle_2::DOMAIN_NAME);
            
            rDn_2_WTSN rdn_2_wtsn(10000, domain_2);
            rdn_2_wtsn.initialize();
            
            // 障害物との交差判定
            std::cout << "cut edges intersecting obstacles" << std::endl;
            rdn_2_wtsn.disconnect_edges(domain_2_ptrs);

            // 重み付け
            std::cout << "weight edges" << std::endl;
            std::vector<Point_2> region_2_vertices {{0.0-50.0, 30.0}, 
                                                    {180.0+50.0, 30.0}, 
                                                    {180.0+50.0, 75.0}, 
                                                    {0.0-50.0, 75.0}};
            Polygon_2 region_2_polygon = Polygon_2(region_2_vertices.begin(), region_2_vertices.end());
            Weight_Passability_WTSN w_region_2 {2.0, 
                                                4, 
                                                15, 
                                                15};
            Region_2_WTSN region_2_wtsn {region_2_polygon, w_region_2};

            std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs {std::make_shared<Region_2_WTSN>(region_2_wtsn)};
            rdn_2_wtsn.weight_edges(region_ptrs);

            // シミュレータの定義
            std::cout << "setup simulator" << std::endl;
            std::shared_ptr<rDn_2_WTSN> rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
            std::vector<Point_2> demand_points {{0.0, 0.0}, 
                                                {180.0, 105.0}};
            std::vector<std::vector<double>> demand_matrix {{0, 1}, 
                                                            {1, 0}};
            Simulator_WTSN simulator (domain_2, 
                                      rdn_2_wtsn_ptr, 
                                      demand_points, 
                                      demand_matrix);

            // // 最短路探索
            // std::cout << "claculate the shortest path tree" << std::endl;
            // std::deque<rDn_2_WTSN::vertex_descriptor> shortest_path;
            // shortest_path = simulator.calc_pedestrian_path(simulator.get_demand_nodes().at(0), 
            //                                                simulator.get_demand_nodes().at(1));
            
            // // 書き出し
            // std::cout << "output the result" << std::endl;
            // for (auto& vertex : shortest_path) {
            //     std::cout << (*simulator.get_net_ptr())[vertex]->x() << ","<< (*simulator.get_net_ptr())[vertex]->y() << ",0.0" << std::endl;
            // }

            // // 減衰
            // std::cout << "damp" << std::endl;
            // simulator.damp(shortest_path);
            // rDn_2_WTSN::edge_iterator eit, eit_end;
            // for (boost::tie(eit, eit_end) = boost::edges(rdn_2_wtsn); eit != eit_end; ++eit) {
            //     double c = std::dynamic_pointer_cast<Edge_2_WTSN>(rdn_2_wtsn[*eit])->access_weight_passability_wtsn().get_curr_count();
            //     if (c != 0) {
            //         std::cout << *eit << " " << c << std::endl;
            //     }
            // }

            // // 増幅
            // std::cout << "amp" << std::endl;
            // simulator.amp();
            // for (boost::tie(eit, eit_end) = boost::edges(rdn_2_wtsn); eit != eit_end; ++eit) {
            //     double c = std::dynamic_pointer_cast<Edge_2_WTSN>(rdn_2_wtsn[*eit])->access_weight_passability_wtsn().get_curr_count();
            //     if (c != 0) {
            //         std::cout << *eit << " " << c << std::endl;
            //     }
            // }

            // セットアップ
            simulator.setup(10, 
                            1000);
            
            // 実行
            std::cout << "execute simulation" << std::endl;
            simulator.run();

            // 記録
            std::cout << "record" << std::endl;
            std::ofstream config_file("/home/workspace/experiment_wtsn/config.json");
            simulator.save_config(config_file);
            config_file.close();

            std::ofstream log_file("/home/workspace/experiment_wtsn/log.csv");
            simulator.save_log(log_file);
            log_file.close();

            std::ofstream f {"/home/workspace/commands.txt"};
            for (auto& edge : simulator.get_living_edges()) {
                    std::shared_ptr<Node_2> s_ptr = (*simulator.get_net_ptr())[edge]->get_source_ptr();
                    std::shared_ptr<Node_2> t_ptr = (*simulator.get_net_ptr())[edge]->get_target_ptr();
                    f << "line" << std::endl;
                    f << s_ptr->x() << "," << s_ptr->y() << ",0.0" << std::endl;
                    f << t_ptr->x() << "," << t_ptr->y() << ",0.0" << std::endl;
            }         

        }
        case 10: {
            //** WTSNの性能評価実験テスト **//
            // ファイルの読み込み
            std::string file_path = "/home/workspace/data/experiment_wtsn/config_3.json";
            std::ifstream input_file(file_path);
            if (!input_file) {
                std::cerr << "Error opening file: " << file_path << std::endl;
                return EXIT_FAILURE;
            }
            nlohmann::json input_json;
            input_file >> input_json;
            input_file.close();

            // 需要点
            std::vector<Point_2> demand_points;
            for (const auto& point : input_json["node_coordinates"]) {
                demand_points.emplace_back(point[0], point[1]);
            }

            // 需要点のバウンディングボックス
            double xmin {std::numeric_limits<double>::max()};
            double xmax {std::numeric_limits<double>::lowest()};
            double ymin {std::numeric_limits<double>::max()};
            double ymax {std::numeric_limits<double>::lowest()};

            for (const auto& point : demand_points) {
                if (point.x() < xmin) xmin = point.x();
                if (point.x() > xmax) xmax = point.x();
                if (point.y() < ymin) ymin = point.y();
                if (point.y() > ymax) ymax = point.y();
            }

            Polygon_2 bbox;
            bbox.push_back(Point_2(xmin, ymin));
            bbox.push_back(Point_2(xmax, ymin));
            bbox.push_back(Point_2(xmax, ymax));
            bbox.push_back(Point_2(xmin, ymax));

            // domain
            double bbox_area = bbox.area();
            double offset_dist = std::sqrt(bbox_area) * 0.05;
            std::vector<boost::shared_ptr<Polygon_2>> offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset_dist, bbox);
            Polygon_2 domain_2 = *(offset_polygons.at(0));

            // ランダムドロネー網
            rDn_2_WTSN rdn_2_wtsn(50000, domain_2);
            rdn_2_wtsn.initialize();

            // 障害物との交差判定
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(domain_2, 
                                                                                                 false, 
                                                                                                 false, 
                                                                                                 true, 
                                                                                                 Obstacle_2::DOMAIN_NAME);
            rdn_2_wtsn.disconnect_edges(domain_2_ptrs);

            // 設定
            Random_Engine::set_seed(17);
            std::vector<double> init_weights {1.001, 1.01, 1.1, 1.2, 1.3, 1.4, 1.5, 2.0, 3.0, 4.0, 5.0};
            // std::reverse(init_weights.begin(), init_weights.end());
            std::vector<size_t> damp_required_counts {5, 10, 15};
            std::vector<size_t> amp_required_counts {15, 30, 45};

            size_t amp_count_standadizer = demand_points.size() * (demand_points.size() - 1);

            for (double init_weight : init_weights) {
                for (size_t damp_required_count : damp_required_counts) {
                    for (size_t amp_required_count : amp_required_counts) {
                        Weight_Passability_WTSN w_region_2 {init_weight, 
                                                            amp_count_standadizer, 
                                                            damp_required_count, 
                                                            amp_required_count};
                        Region_2_WTSN region_2_wtsn {w_region_2};
                        std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs {std::make_shared<Region_2_WTSN>(region_2_wtsn)};
                        rdn_2_wtsn.weight_edges(region_ptrs);

                        // シミュレータの定義
                        std::shared_ptr<rDn_2_WTSN> rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
                        Simulator_WTSN simulator (domain_2, 
                                                rdn_2_wtsn_ptr, 
                                                "/home/workspace/data/experiment_wtsn/config_3.json");
                        
                        // セットアップ
                        simulator.setup(amp_count_standadizer, 
                                        1000);
                        
                        // 実行
                        simulator.run();

                        // 記録
                        std::ofstream config_file("/home/workspace/data/experiment_wtsn/config_3_.json");
                        simulator.save_config(config_file);
                        config_file.close();

                        std::ofstream log_file("/home/workspace/data/experiment_wtsn/log_3.csv");
                        simulator.save_log(log_file);
                        log_file.close();

                        std::ofstream f {"/home/workspace/wtsn_" + 
                                         std::to_string(init_weight) + 
                                         "_" +
                                         std::to_string(damp_required_count) + 
                                         "_" +
                                         std::to_string(amp_required_count) + 
                                         "_" + 
                                         ".txt"};
                        for (auto& point : demand_points) {
                            f << "point" << std::endl;
                            f << point.x() << "," << point.y() << ",0.0" << std::endl;
                        }
                        for (auto& edge : simulator.get_living_edges()) {
                            std::shared_ptr<Node_2> s_ptr = (*simulator.get_net_ptr())[edge]->get_source_ptr();
                            std::shared_ptr<Node_2> t_ptr = (*simulator.get_net_ptr())[edge]->get_target_ptr();
                            f << "line" << std::endl;
                            f << s_ptr->x() << "," << s_ptr->y() << ",0.0" << std::endl;
                            f << t_ptr->x() << "," << t_ptr->y() << ",0.0" << std::endl;
                        }   
                        
                        Net_2 simplified_network = simulator.simplify_wtsn();

                        std::ofstream g {"/home/workspace/result_network_" + 
                                         std::to_string(init_weight) + 
                                         "_" +
                                         std::to_string(damp_required_count) + 
                                         "_" +
                                         std::to_string(amp_required_count) + 
                                         "_" + 
                                         ".txt"};
                        Net_2::edge_iterator eit, eit_end;
                        for (boost::tie(eit, eit_end) = boost::edges(simplified_network); eit != eit_end; ++eit) {
                            auto src = boost::source(*eit, simplified_network);
                            auto tgt = boost::target(*eit, simplified_network);
                            auto src_p_ptr = simplified_network[src];
                            auto tgt_p_ptr = simplified_network[tgt];
                            g << "line" << std::endl;
                            g << src_p_ptr->x() << "," << src_p_ptr->y() << ",0.0" << std::endl;
                            g << tgt_p_ptr->x() << "," << tgt_p_ptr->y() << ",0.0" << std::endl;
                        }

                        if (!simulator.is_connecting_all_demand_nodes()) {
                            std::cout << std::scientific 
                                    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                                    << init_weight << "," 
                                    << damp_required_count << "," 
                                    << amp_required_count << ","
                                    << "" << "," 
                                    << "" << std::endl;
                        } else {
                            double total_detour = simulator.calc_total_detour(simplified_network, true);
                            double total_length = simulator.calc_total_length(simplified_network);

                            std::cout << std::scientific 
                                    << std::setprecision(std::numeric_limits<double>::max_digits10) 
                                    << init_weight << "," 
                                    << damp_required_count << "," 
                                    << amp_required_count << ","
                                    << total_length << "," 
                                    << total_detour << std::endl;
                        }

                    }
                }
            }
            
        }
        case 11: {
            //** WTSNの性能評価実験 **//
            std::vector<size_t> case_nums {58};
            // for (size_t case_num {0}; case_num < 100; case_num++) {
            for (size_t case_num : case_nums) {
                // ファイルの読み込み
                std::string ifile_path = "/home/workspace/data/experiment_wtsn/terminal5/beta0.5/config_" + std::to_string(case_num) + ".json";
                std::ifstream input_file(ifile_path);
                if (!input_file) {
                    std::cerr << "Error opening file: " << ifile_path << std::endl;
                    continue;
                }
                nlohmann::json input_json;
                input_file >> input_json;
                input_file.close();

                std::string ofile_path = "/home/workspace/data/experiment_wtsn/terminal5/beta0.5/result_" + std::to_string(case_num) + ".csv";
                std::ofstream output_file(ofile_path, std::ios::app);

                // 需要点
                std::vector<Point_2> demand_points;
                for (const auto& point : input_json["node_coordinates"]) {
                    demand_points.emplace_back(point[0], point[1]);
                }

                // 需要点のバウンディングボックス
                double xmin {std::numeric_limits<double>::max()};
                double xmax {std::numeric_limits<double>::lowest()};
                double ymin {std::numeric_limits<double>::max()};
                double ymax {std::numeric_limits<double>::lowest()};

                for (const auto& point : demand_points) {
                    if (point.x() < xmin) xmin = point.x();
                    if (point.x() > xmax) xmax = point.x();
                    if (point.y() < ymin) ymin = point.y();
                    if (point.y() > ymax) ymax = point.y();
                }

                Polygon_2 bbox;
                bbox.push_back(Point_2(xmin, ymin));
                bbox.push_back(Point_2(xmax, ymin));
                bbox.push_back(Point_2(xmax, ymax));
                bbox.push_back(Point_2(xmin, ymax));

                // domain
                double bbox_area = bbox.area();
                double offset_dist = std::sqrt(bbox_area) * 0.05;
                std::vector<boost::shared_ptr<Polygon_2>> offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset_dist, bbox);
                Polygon_2 domain_2 = *(offset_polygons.at(0));

                // ランダムドロネー網
                rDn_2_WTSN rdn_2_wtsn(50000, domain_2);
                rdn_2_wtsn.initialize();

                // 障害物との交差判定
                std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(domain_2, 
                                                                                                    false, 
                                                                                                    false, 
                                                                                                    true, 
                                                                                                    Obstacle_2::DOMAIN_NAME);
                rdn_2_wtsn.disconnect_edges(domain_2_ptrs);

                // 設定
                std::uniform_int_distribution<size_t> seed_dist(0, SIZE_MAX);
                size_t trial_num {3};
                // std::vector<double> init_weights {1.001, 1.005, 1.01, 1.1, 1.25, 1.5, 2.0, 2.5, 4.0};
                // std::vector<double> init_weights {1.05, 1.2, 1.75, 3.0, 5.0};
                std::vector<double> init_weights {1.001, 1.005, 1.01, 1.05, 1.1, 1.2, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0};
                std::vector<size_t> required_counts {5, 10, 15, 30, 50};

                size_t amp_count_standadizer = demand_points.size() * (demand_points.size() - 1);

                std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

                for (double init_weight : init_weights) {
                    for (size_t d_i {0}; d_i < required_counts.size(); d_i++) {
                        size_t damp_required_count = required_counts.at(d_i);
                        for (size_t a_i {d_i}; a_i < required_counts.size(); a_i++) {
                            size_t amp_required_count = required_counts.at(a_i);
                            for (size_t trial {0}; trial < trial_num; trial++) {
                                Weight_Passability_WTSN w_region_2 {init_weight, 
                                                                    amp_count_standadizer, 
                                                                    damp_required_count, 
                                                                    amp_required_count};
                                Region_2_WTSN region_2_wtsn {w_region_2};
                                std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs {std::make_shared<Region_2_WTSN>(region_2_wtsn)};
                                rdn_2_wtsn.weight_edges(region_ptrs);

                                // シミュレータの定義
                                std::shared_ptr<rDn_2_WTSN> rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
                                Simulator_WTSN simulator (domain_2, 
                                                        rdn_2_wtsn_ptr, 
                                                        ifile_path);
                                
                                // セットアップ
                                simulator.setup(amp_count_standadizer, 
                                                1000);
                                
                                // 実行
                                size_t seed = seed_dist(Random_Engine::get_engine());
                                Random_Engine::set_seed(seed);
                                simulator.run();

                                // 記録                                
                                Net_2 simplified_network = simulator.simplify_wtsn();

                                if (!simulator.is_connecting_all_demand_nodes()) {
                                    output_file << std::scientific 
                                            << std::setprecision(std::numeric_limits<double>::max_digits10) 
                                            << seed << "," 
                                            << init_weight << "," 
                                            << damp_required_count << "," 
                                            << amp_required_count << ","
                                            << "" << "," 
                                            << "" << std::endl;
                                } else {
                                    // double total_detour = simulator.calc_total_detour(simplified_network, true);
                                    // double total_length = simulator.calc_total_length(simplified_network);

                                    std::pair <double, double> total_length_and_detour = simulator.evaluate_network(simplified_network, 
                                                                                                                       false, 
                                                                                                                       true, 
                                                                                                                       {}, 
                                                                                                                       true);
                                    double total_length = total_length_and_detour.first;
                                    double total_detour = total_length_and_detour.second;

                                    output_file << std::scientific 
                                            << std::setprecision(std::numeric_limits<double>::max_digits10) 
                                            << seed << "," 
                                            << init_weight << "," 
                                            << damp_required_count << "," 
                                            << amp_required_count << ","
                                            << total_length << "," 
                                            << total_detour << std::endl;
                                }
                            }                           
                        }
                    }
                }

                std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end_time - start_time;

                output_file.close();
                std::cout << "Finished case " << case_num << " in " << elapsed_seconds.count() << " seconds)" << std::endl;

            }

            break;
            
        }
        case 12: {
            //** WTSNの性能評価実験 **//
            const std::string case_num_str = argv[2];
            const std::string seed_str = argv[3];
            const std::string init_weight_str = argv[4];
            const std::string damp_count_str = argv[5];
            const std::string amp_count_str = argv[6];

            const size_t case_num = std::stoul(case_num_str);
            const size_t seed = std::stoul(seed_str);
            const double init_weight = std::stod(init_weight_str);
            const size_t damp_required_count = std::stoul(damp_count_str);
            const size_t amp_required_count = std::stoul(amp_count_str);
            
            // ファイルの読み込み
            std::string ifile_path = "/home/workspace/data/experiment_wtsn/terminal5/beta2/config_" + std::to_string(case_num) + ".json";
            std::ifstream input_file(ifile_path);
            if (!input_file) {
                std::cerr << "Error opening file: " << ifile_path << std::endl;
                return EXIT_FAILURE;
            }
            nlohmann::json input_json;
            input_file >> input_json;
            input_file.close();

            std::string nwconfig_file_path = "/home/workspace/nwconfig_" + std::to_string(case_num) + ".txt";
            std::ofstream nwconfig_file(nwconfig_file_path, std::ios::app);
            std::string log_file_path = "/home/workspace/log_" + std::to_string(case_num) + ".txt";
            std::ofstream log_file(log_file_path, std::ios::app);

            // 需要点
            std::vector<Point_2> demand_points;
            for (const auto& point : input_json["node_coordinates"]) {
                demand_points.emplace_back(point[0], point[1]);
            }

            // 需要点のバウンディングボックス
            double xmin {std::numeric_limits<double>::max()};
            double xmax {std::numeric_limits<double>::lowest()};
            double ymin {std::numeric_limits<double>::max()};
            double ymax {std::numeric_limits<double>::lowest()};

            for (const auto& point : demand_points) {
                if (point.x() < xmin) xmin = point.x();
                if (point.x() > xmax) xmax = point.x();
                if (point.y() < ymin) ymin = point.y();
                if (point.y() > ymax) ymax = point.y();
            }

            Polygon_2 bbox;
            bbox.push_back(Point_2(xmin, ymin));
            bbox.push_back(Point_2(xmax, ymin));
            bbox.push_back(Point_2(xmax, ymax));
            bbox.push_back(Point_2(xmin, ymax));

            // domain
            double bbox_area = bbox.area();
            double offset_dist = std::sqrt(bbox_area) * 0.05;
            std::vector<boost::shared_ptr<Polygon_2>> offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset_dist, bbox);
            Polygon_2 domain_2 = *(offset_polygons.at(0));

            // ランダムドロネー網
            rDn_2_WTSN rdn_2_wtsn(50000, domain_2);
            rdn_2_wtsn.initialize();

            // 障害物との交差判定
            std::vector<std::shared_ptr<Obstacle_2>> domain_2_ptrs = Obstacle_2::convert_polygon(domain_2, 
                                                                                                false, 
                                                                                                false, 
                                                                                                true, 
                                                                                                Obstacle_2::DOMAIN_NAME);
            rdn_2_wtsn.disconnect_edges(domain_2_ptrs);

            // 設定
            size_t amp_count_standadizer = demand_points.size() * (demand_points.size() - 1);

            std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

            Weight_Passability_WTSN w_region_2 {init_weight, 
                                                amp_count_standadizer, 
                                                damp_required_count, 
                                                amp_required_count};
            Region_2_WTSN region_2_wtsn {w_region_2};
            std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs {std::make_shared<Region_2_WTSN>(region_2_wtsn)};
            rdn_2_wtsn.weight_edges(region_ptrs);

            // シミュレータの定義
            std::shared_ptr<rDn_2_WTSN> rdn_2_wtsn_ptr = std::make_shared<rDn_2_WTSN>(rdn_2_wtsn);
            Simulator_WTSN simulator (domain_2, 
                                    rdn_2_wtsn_ptr, 
                                    ifile_path);
            
            // セットアップ
            simulator.setup(amp_count_standadizer, 
                            1000);
            
            // 実行
            Random_Engine::set_seed(seed);
            simulator.run();

            // 記録                                
            Net_2 simplified_network = simulator.simplify_wtsn();  

            std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end_time - start_time;
            std::cout << "Finished case " << case_num << " in " << elapsed_seconds.count() << " seconds)" << std::endl;

            Net_2::edge_iterator eit, eit_end;
            for (boost::tie(eit, eit_end) = boost::edges(simplified_network); eit != eit_end; ++eit) {
                auto src = boost::source(*eit, simplified_network);
                auto tgt = boost::target(*eit, simplified_network);
                auto src_p_ptr = simplified_network[src];
                auto tgt_p_ptr = simplified_network[tgt];
                nwconfig_file << "line" << std::endl;
                nwconfig_file << src_p_ptr->x() << "," << src_p_ptr->y() << ",0.0" << std::endl;
                nwconfig_file << tgt_p_ptr->x() << "," << tgt_p_ptr->y() << ",0.0" << std::endl;
            }
            // for (auto& edge : simulator.get_activated_edges()) {
            // // for (auto& edge : simulator.get_living_edges()) {
            //         std::shared_ptr<Node_2> s_ptr = (*simulator.get_net_ptr())[edge]->get_source_ptr();
            //         std::shared_ptr<Node_2> t_ptr = (*simulator.get_net_ptr())[edge]->get_target_ptr();
            //         nwconfig_file << "line" << std::endl;
            //         nwconfig_file << s_ptr->x() << "," << s_ptr->y() << ",0.0" << std::endl;
            //         nwconfig_file << t_ptr->x() << "," << t_ptr->y() << ",0.0" << std::endl;
            // }     

            nwconfig_file.close();

            simulator.save_log(log_file);
            log_file.close();
            
            if (simulator.is_connecting_all_demand_nodes()) {
                std::cout << "Evaluation" << std::endl;
                std::pair <double, double> total_length_and_detour = simulator.evaluate_network(simplified_network, 
                                                                                                    false, 
                                                                                                    true, 
                                                                                                    {}, 
                                                                                                    true);
                double total_length = total_length_and_detour.first;
                double total_detour = total_length_and_detour.second;

                std::cout << std::scientific 
                        << std::setprecision(std::numeric_limits<double>::max_digits10) 
                        << seed << "," 
                        << init_weight << "," 
                        << damp_required_count << "," 
                        << amp_required_count << ","
                        << total_length << "," 
                        << total_detour << std::endl;
            }

            break;
            
        }

        default:
            break;
    }

    return EXIT_SUCCESS;

}