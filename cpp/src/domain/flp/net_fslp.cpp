// include header
#include "domain/flp/net_fslp.h"

// include stl util
#include "core/util/std_vector_util.h"
#include "core/util/random_engine.h"

//** Static Member Variable **//
static std::uniform_real_distribution<double> dist(0.0, 1.0);
const Point_2 Net_FSLP::DUMMY_POINT_FACILITIES = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));
const Point_2 Net_FSLP::DUMMY_POINT_SIGNS = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));
const Point_2 Net_FSLP::DUMMY_POINT_MAIN_BUILDINGS = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));

//** Constructor **//
Net_FSLP::Net_FSLP() : net_ptr(std::make_shared<Net_2>()) {} 
Net_FSLP::Net_FSLP(std::shared_ptr<Net_2> net_ptr) : net_ptr(net_ptr) {
    if (!this->net_ptr) {
        throw std::runtime_error("net_ptr is not initialized.\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

//** Getter **//
Point_2 Net_FSLP::get_inner_point() const {
    return inner_point;
}
double Net_FSLP::get_visible_length() const {
    return visible_length;
}
std::vector<Net_2::vertex_descriptor> Net_FSLP::get_demands() const {
    return demands;
}
std::vector<Net_2::vertex_descriptor> Net_FSLP::get_facilities() const {
    return  facilities;
}
std::vector<Net_2::vertex_descriptor> Net_FSLP::get_signs() const {
    return signs;
}
std::vector<Net_2::vertex_descriptor> Net_FSLP::get_main_buildings() const {
    return main_buildings;
}
Net_2::vertex_descriptor Net_FSLP::get_dummy_vertex_facilities() const {
    return dummy_vertex_facilities;
}
Net_2::vertex_descriptor Net_FSLP::get_dummy_vertex_signs() const {
    return dummy_vertex_signs;
}
Net_2::vertex_descriptor Net_FSLP::get_dummy_vertex_main_buildings() const {
    return dummy_vertex_main_buildings;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_FSLP::get_facility_coverage_tree() const {
    return facility_coverage_tree;
}
std::unordered_map<Net_2::vertex_descriptor, 
                   std::vector<std::pair<Net_2::vertex_descriptor, double>>
> Net_FSLP::get_facility_coverage_trees() const {
    return facility_coverage_trees;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_FSLP::get_facility_shortest_path_tree() const {
    return facility_shortest_path_tree;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_FSLP::get_sign_coverage_tree() const {
    return sign_coverage_tree;
}
std::unordered_map<Net_2::vertex_descriptor, 
                   std::vector<std::pair<Net_2::vertex_descriptor, double>>
> Net_FSLP::get_sign_coverage_trees() const {
    return sign_coverage_trees;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_FSLP::get_main_building_shortest_path_tree() const {
    return main_building_shortest_path_tree;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_FSLP::get_facility_assignment_to_demand() const {
    return facility_assignment_to_demand;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_FSLP::get_sign_assignment_to_demand() const {
    return sign_assignment_to_demand;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_FSLP::get_facility_assignment_to_sign() const {
    return facility_assignment_to_sign;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_FSLP::get_facility_assignment_to_main_building() const {
    return facility_assignment_to_main_building;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_FSLP::get_main_building_assignment_to_demand() const {
    return main_building_assignment_to_demand;
}

//** Setter **//
void Net_FSLP::set_inner_point(const Point_2 inner_point) {
    this->inner_point = inner_point;
}
void Net_FSLP::set_visible_length(const double visible_length) {
    this->visible_length = visible_length;
}
void Net_FSLP::set_demands() {
    std::unordered_set<Net_2::vertex_descriptor> demands_set;
    demands_set = this->net_ptr->calculate_reachable_vertices(this->inner_point);
    this->demands = std::vector<Net_2::vertex_descriptor>(demands_set.begin(), demands_set.end());
}
void Net_FSLP::set_demands(const Point_2 inner_point) {
    set_inner_point(inner_point);
    set_demands();
}
void Net_FSLP::set_demands(const std::vector<Net_2::vertex_descriptor> demands) {
    this->demands = demands;
}
void Net_FSLP::set_facilities(const std::vector<Net_2::vertex_descriptor> facilities) {
    this->facilities = facilities;
}
void Net_FSLP::set_signs(const std::vector<Net_2::vertex_descriptor> signs) {
    this->signs = signs;
}
void Net_FSLP::set_main_buildings(const std::vector<Net_2::vertex_descriptor> main_buildings) {
    this->main_buildings = main_buildings;
}
void Net_FSLP::set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities) {
    this->dummy_vertex_facilities = dummy_vertex_facilities;
}
void Net_FSLP::set_dummy_vertex_signs(const Net_2::vertex_descriptor dummy_vertex_signs) {
    this->dummy_vertex_signs = dummy_vertex_signs;
}
void Net_FSLP::set_dummy_vertex_main_buildings(const Net_2::vertex_descriptor dummy_vertex_main_buildings) {
    this->dummy_vertex_main_buildings = dummy_vertex_main_buildings;
}

//** Network Methods **//
void Net_FSLP::build_trees() {
    // build_facility_coverage_tree();
    build_facility_coverage_trees();
    build_facility_shortest_path_tree();
    // build_sign_coverage_tree();
    build_sign_coverage_trees();
    build_main_building_shortest_path_tree();
}

void Net_FSLP::clear_trees() {
    facility_coverage_tree.clear();
    facility_shortest_path_tree.clear();
    sign_coverage_tree.clear();
    main_building_shortest_path_tree.clear();
}

void Net_FSLP::insert_dummy_vertex_facilities() {
    
    std::shared_ptr<Node_2> dummy_node_facilities_ptr = std::make_shared<Node_2>(DUMMY_POINT_FACILITIES);

    Net_2::vertex_descriptor dummy_vertex_facilities = boost::add_vertex(dummy_node_facilities_ptr, *(this->net_ptr));
    this->set_dummy_vertex_facilities(dummy_vertex_facilities);

    for (const auto& facility_vertex : this->facilities) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], (*(this->net_ptr))[facility_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_facilities, facility_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_FSLP::insert_dummy_vertex_signs() {
    
    std::shared_ptr<Node_2> dummy_node_signs_ptr = std::make_shared<Node_2>(DUMMY_POINT_SIGNS);

    Net_2::vertex_descriptor dummy_vertex_signs = boost::add_vertex(dummy_node_signs_ptr, *(this->net_ptr));
    this->set_dummy_vertex_signs(dummy_vertex_signs);

    for (const auto& sign_vertex : this->signs) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_signs], (*(this->net_ptr))[sign_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_signs, sign_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_FSLP::insert_dummy_vertex_main_buildings() {
    
    std::shared_ptr<Node_2> dummy_node_main_buildings_ptr = std::make_shared<Node_2>(DUMMY_POINT_MAIN_BUILDINGS);

    Net_2::vertex_descriptor dummy_vertex_main_buildings = boost::add_vertex(dummy_node_main_buildings_ptr, *(this->net_ptr));
    this->set_dummy_vertex_main_buildings(dummy_vertex_main_buildings);

    for (const auto& main_building_vertex : this->main_buildings) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_main_buildings], (*(this->net_ptr))[main_building_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_main_buildings, main_building_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_FSLP::remove_dummy_vertices() {
    std::vector<Net_2::vertex_descriptor> to_remove = {
        dummy_vertex_facilities,
        dummy_vertex_signs,
        dummy_vertex_main_buildings
    };

    // 削除対象の頂点を大きい順にソート（削除時のインデックスずれを防ぐ）
    std::sort(to_remove.rbegin(), to_remove.rend());

    for (int v : to_remove) {
        if (v != UNINITIALIZED_DUMMY_VERTEX_FACILITIES && 
            v != UNINITIALIZED_DUMMY_VERTEX_SIGNS && 
            v != UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS) {
            this->net_ptr->remove_vertex(v);
        }
    }

    // 無効な値にリセット
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;
    dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;
    dummy_vertex_main_buildings = UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS;
}

void Net_FSLP::remove_dummy_vertex_facilities() {
    boost::clear_vertex(dummy_vertex_facilities, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_facilities, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;
}

void Net_FSLP::remove_dummy_vertex_signs() {
    boost::clear_vertex(dummy_vertex_signs, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_signs, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;
}

void Net_FSLP::remove_dummy_vertex_main_buildings() {
    boost::clear_vertex(dummy_vertex_main_buildings, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_main_buildings, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_main_buildings = UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS;
}

void Net_FSLP::update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                              const Net_2::vertex_descriptor next_facility_vertex) 
{
    boost::remove_edge(dummy_vertex_facilities, prev_facility_vertex, *(this->net_ptr));

    std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], 
                                                                      (*(this->net_ptr))[next_facility_vertex]);
    dummy_edge_ptr->set_is_dummy(true);
    boost::add_edge(this->dummy_vertex_facilities, next_facility_vertex, dummy_edge_ptr, *(this->net_ptr));
}

void Net_FSLP::update_dummy_vertex_signs(const Net_2::vertex_descriptor prev_sign_vertex, 
                                         const Net_2::vertex_descriptor next_sign_vertex) 
{
    boost::remove_edge(dummy_vertex_signs, prev_sign_vertex, *(this->net_ptr));

    std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_signs], 
                                                                      (*(this->net_ptr))[next_sign_vertex]);
    dummy_edge_ptr->set_is_dummy(true);
    boost::add_edge(this->dummy_vertex_signs, next_sign_vertex, dummy_edge_ptr, *(this->net_ptr));
}

void Net_FSLP::build_facility_coverage_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_signs, dummy_vertex_main_buildings};
    this->facility_coverage_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_facilities(), 
                                                                               Net_2::MODE_VISIBILITY, 
                                                                               false, 
                                                                               true, 
                                                                               prohibited_vertices);
}

void Net_FSLP::build_facility_coverage_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_main_buildings};
    
    // 未登録のサービス供給点
    for (const auto& facility : this->facilities) {
        if (this->facility_coverage_trees.find(facility) == this->facility_coverage_trees.end()) {       
            this->facility_coverage_trees[facility] = this->net_ptr->calculate_shortest_path_tree(facility, 
                                                                                                Net_2::MODE_VISIBILITY, 
                                                                                                false, 
                                                                                                true, 
                                                                                                prohibited_vertices);
        }
    }

    // サービス供給点でなくなったもの
    for (auto it = this->facility_coverage_trees.begin(); it != this->facility_coverage_trees.end(); ) {
        if (std::find(this->facilities.begin(), this->facilities.end(), it->first) == this->facilities.end()) {
            it = this->facility_coverage_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }

}

void Net_FSLP::build_facility_shortest_path_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_signs, dummy_vertex_main_buildings};
    this->facility_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_facilities(), 
                                                                                    Net_2::MODE_ROUTE, 
                                                                                    true, 
                                                                                    true, 
                                                                                    prohibited_vertices);
}

void Net_FSLP::build_sign_coverage_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, dummy_vertex_main_buildings};
    
    // 未登録のサイン
    for (const auto& sign : this->signs) {
        if (this->sign_coverage_trees.find(sign) == this->sign_coverage_trees.end()) {       
            this->sign_coverage_trees[sign] = this->net_ptr->calculate_shortest_path_tree(sign, 
                                                                                       Net_2::MODE_VISIBILITY, 
                                                                                       false, 
                                                                                       true, 
                                                                                       prohibited_vertices);
        }
    }

    // サインでなくなったもの
    for (auto it = this->sign_coverage_trees.begin(); it != this->sign_coverage_trees.end(); ) {
        if (std::find(this->signs.begin(), this->signs.end(), it->first) == this->signs.end()) {
            it = this->sign_coverage_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }
    
}

void Net_FSLP::build_sign_coverage_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
        dummy_vertex_signs, 
        dummy_vertex_main_buildings};

    for (const auto& sign : this->signs) {
        this->sign_coverage_trees[sign] = this->net_ptr->calculate_shortest_path_tree(sign, 
                                                                                      Net_2::MODE_VISIBILITY, 
                                                                                      false, 
                                                                                      true, 
                                                                                      prohibited_vertices);
    }

}

void Net_FSLP::build_main_building_shortest_path_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, dummy_vertex_signs};
    this->main_building_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_main_buildings(), 
                                                                                         Net_2::MODE_ROUTE, 
                                                                                         true, 
                                                                                         true, 
                                                                                         prohibited_vertices);
}

std::deque<Net_2::vertex_descriptor> Net_FSLP::calculate_path_to_main_building(Net_2::vertex_descriptor demand) const {
    std::deque<Net_2::vertex_descriptor> path_to_main_building;
    std::pair<bool, Net_2::vertex_descriptor> main_building = this->get_assigned_main_building_to_demand(demand);
    
    if (!main_building.first) {
        throw std::runtime_error(
            "Demand has to have assigned main building.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
    for (auto& vertex=demand; vertex != main_building.second; vertex = this->main_building_shortest_path_tree.at(vertex).first) {
        path_to_main_building.push_back(vertex);
    }

    path_to_main_building.push_back(main_building.second);
    
    return path_to_main_building;
}

//** Initialization Methods **//
Net_2::vertex_descriptor Net_FSLP::search_nearest_demand(const Point_2 point) const {
    Net_2::vertex_descriptor nearest_vertex = net_ptr->find_nearest_node(point);

    if (find_index(demands, nearest_vertex) != -1) {
        // pに最近隣の頂点が需要点の中にある
        return nearest_vertex;
    } else {
        // pに最近隣の頂点が需要点の中にない
        // 最近隣の頂点から順に需要点を探索する
        std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                                   dummy_vertex_signs, 
                                                                   dummy_vertex_main_buildings};
        std::vector<std::pair<Net_2::vertex_descriptor, double>> spt = 
            net_ptr->calculate_shortest_path_tree(nearest_vertex, 
                                                  Net_2::MODE_ROUTE, 
                                                  false, 
                                                  false, 
                                                  prohibited_vertices);
        
        // 最短経路木上の距離の昇順でソート
        //!ユークリッド距離の順序とは一致しない
        std::sort(spt.begin(), spt.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        for (const auto& v_and_d : spt) {
            
            if (find_index(demands, v_and_d.first) != -1) {
                return v_and_d.first;
            }
        }

        std::cerr << "Could not find nearest demand to " << point << std::endl;
        throw std::runtime_error(
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );

    }
}

std::vector<Net_2::vertex_descriptor> Net_FSLP::search_nearest_demands(const std::vector<Point_2> points) const {
    std::vector<Net_2::vertex_descriptor> nearest_demands {};
    for (const auto& p : points) {
        nearest_demands.push_back(search_nearest_demand(p));
    }

    return nearest_demands;
    
}

std::vector<Net_2::vertex_descriptor> Net_FSLP::select_demands_at_random(const size_t selected_num) {
    // 重複無しの無作為抽出
    return random_sampling(demands, selected_num, false);
}

void Net_FSLP::initialize_facilities(const std::vector<Point_2> facility_points) {

    this->facilities = search_nearest_demands(facility_points);

    if (this->dummy_vertex_facilities == UNINITIALIZED_DUMMY_VERTEX_FACILITIES) {
        this->insert_dummy_vertex_facilities();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing facilities.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }    

}

void Net_FSLP::initialize_facilities(const size_t facility_num) {

    if (facility_num == 0) {
        this->facilities = std::vector<Net_2::vertex_descriptor>();
    } else {
        this->facilities = select_demands_at_random(facility_num);
    }

    if (this->dummy_vertex_facilities == UNINITIALIZED_DUMMY_VERTEX_FACILITIES) {
        this->insert_dummy_vertex_facilities();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing facilities.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

}

void Net_FSLP::initialize_signs(const std::vector<Point_2> sign_points) {

    this->signs = search_nearest_demands(sign_points);

    if (this->dummy_vertex_signs == UNINITIALIZED_DUMMY_VERTEX_SIGNS) {
        this->insert_dummy_vertex_signs();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing signs.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
}

void Net_FSLP::initialize_signs(const size_t sign_num) {

    if (sign_num == 0) {
        this->signs = std::vector<Net_2::vertex_descriptor>();
    } else {
        this->signs = select_demands_at_random(sign_num);
    }

    if (this->dummy_vertex_signs == UNINITIALIZED_DUMMY_VERTEX_SIGNS) {
        this->insert_dummy_vertex_signs();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing signs.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
}

void Net_FSLP::initialize_main_buildings(const std::vector<Point_2> main_building_points) {

    this->main_buildings = search_nearest_demands(main_building_points);

    if (this->dummy_vertex_main_buildings == UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS) {
        this->insert_dummy_vertex_main_buildings();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing main buildings.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

}

//** Finalization Methods **//
void Net_FSLP::clear() {
    remove_dummy_vertices();
}

//** Constraint Method **//
std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::get_assigned_facility_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->facility_assignment_to_demand.find(demand) != this->facility_assignment_to_demand.end()) {
        return std::make_pair(true, this->facility_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::get_assigned_sign_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->sign_assignment_to_demand.find(demand) != this->sign_assignment_to_demand.end()) {
        return std::make_pair(true, this->sign_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::get_assigned_facility_to_sign(Net_2::vertex_descriptor sign) const {
    if (this->facility_assignment_to_sign.find(sign) != this->facility_assignment_to_sign.end()) {
        return std::make_pair(true, this->facility_assignment_to_sign.at(sign));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::get_assigned_facility_to_main_building(Net_2::vertex_descriptor main_building) const {
    if (this->facility_assignment_to_main_building.find(main_building) != this->facility_assignment_to_main_building.end()) {
        return std::make_pair(true, this->facility_assignment_to_main_building.at(main_building));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::get_assigned_main_building_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->main_building_assignment_to_demand.find(demand) != this->main_building_assignment_to_demand.end()) {
        return std::make_pair(true, this->main_building_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

void Net_FSLP::build_assignments() {
    assign_facility_to_demand();
    assign_sign_to_demand();
    assign_facility_to_sign();
    assign_facility_to_main_building();
    assign_main_building_to_demand();

    update_facility_assignment_to_demand();
}

void Net_FSLP::clear_assignments() {
    facility_assignment_to_demand.clear();
    sign_assignment_to_demand.clear();
    facility_assignment_to_sign.clear();
    facility_assignment_to_main_building.clear();
    main_building_assignment_to_demand.clear();
}

void Net_FSLP::assign_facility_to_demand() {
    // サービス供給点の需要点に対する割当＝可視かつ最寄り
    std::unordered_map<Net_2::vertex_descriptor, double> vis_min_lens {}; // サービス供給点までの可視長さ
    for (const auto& facility : this->facilities) {
        std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->facility_coverage_trees[facility];
    
        // 最短経路木を反転する
        // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
        std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;
    
        for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
            Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
            if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_FACILITIES) {
                assignment_tree_r[parent].push_back(i);
            }
        }
    
        // サービス供給点を需要点に割当
        // サービス供給点から見える頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> visible_vertices;
        visible_vertices.insert(facility);
        this->net_ptr->search_visible_vertices(facility, 
                                                assignment_tree, 
                                                assignment_tree_r,
                                                visible_vertices, 
                                                this->visible_length);
        
        // 割当
        for (const auto& visible_vertex : visible_vertices) {
            if (find_index(this->demands, visible_vertex) != -1) {
                // 可視である需要点
                double vis_len = assignment_tree[visible_vertex].second; // 可視長さを取得
                bool update = false;
                if (this->facility_assignment_to_demand.find(visible_vertex) == this->facility_assignment_to_demand.end()) {
                    // 未割当
                    update = true;
                } else if (vis_len < vis_min_lens[visible_vertex]) {
                    // 可視長さが短くなる
                    update = true;
                }

                if (update) {
                    this->facility_assignment_to_demand[visible_vertex] = facility;
                    vis_min_lens[visible_vertex] = vis_len;
                }
            }
        }

    }

}

void Net_FSLP::assign_sign_to_demand() {
    // サインの需要点に対する割当＝可視かつ最寄り
    std::unordered_map<Net_2::vertex_descriptor, double> vis_min_lens; // サービス供給点までの可視長さ
    for (const auto& sign : this->signs) {
        std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->sign_coverage_trees[sign];
    
        // 最短経路木を反転する
        // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
        std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;
    
        for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
            Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
            if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_SIGNS) {
                assignment_tree_r[parent].push_back(i);
            }
        }
    
        // サインを需要点に割当
        // サインから見える頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> visible_vertices;
        visible_vertices.insert(sign);
        this->net_ptr->search_visible_vertices(sign, 
                                               assignment_tree, 
                                               assignment_tree_r,
                                               visible_vertices, 
                                               this->visible_length);
        
        // 割当
        for (const auto& visible_vertex : visible_vertices) {
            if (find_index(this->demands, visible_vertex) != -1) {
                // 可視である需要点
                double vis_len = assignment_tree[visible_vertex].second; // 可視長さを取得
                bool update = false;
                if (this->sign_assignment_to_demand.find(visible_vertex) == this->sign_assignment_to_demand.end()) {
                    // 未割当
                    update = true;
                } else if (vis_len < vis_min_lens[visible_vertex]) {
                    // 可視長さが短くなる
                    update = true;
                }

                if (update) {
                    this->sign_assignment_to_demand[visible_vertex] = sign;
                    vis_min_lens[visible_vertex] = vis_len;
                }
                
            }
        }

    }
}

void Net_FSLP::assign_facility_to_sign() {
    // サービス供給点のサインに対する割当＝最寄り
    std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->facility_shortest_path_tree;

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;

    for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
        Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
        if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_FACILITIES) {
            assignment_tree_r[parent].push_back(i);
        }
    }

    // サービス供給点をサインに割当
    for (const auto& facility: this->facilities) {
        // サービス供給点から最も近い頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> reachable_vertices;
        reachable_vertices.insert(facility);
        this->net_ptr->search_reachable_vertices(facility, 
                                                 assignment_tree, 
                                                 assignment_tree_r,
                                                 reachable_vertices, 
                                                 std::numeric_limits<double>::max());
        
        // 割当
        for (const auto& sign : this->signs) {
            if (reachable_vertices.find(sign) != reachable_vertices.end()) {
                this->facility_assignment_to_sign[sign] = facility;
            }
        }

    }

}

void Net_FSLP::assign_facility_to_main_building() {
    // サービス供給点の主要建物に対する割当＝最寄り
    std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->facility_shortest_path_tree;

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;

    for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
        Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
        if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_FACILITIES) {
            assignment_tree_r[parent].push_back(i);
        }
    }

    // サービス供給点を主要建物に割当
    for (const auto& facility: this->facilities) {
        // サービス供給点から最も近い頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> reachable_vertices;
        reachable_vertices.insert(facility);
        this->net_ptr->search_reachable_vertices(facility, 
                                                 assignment_tree, 
                                                 assignment_tree_r,
                                                 reachable_vertices, 
                                                 std::numeric_limits<double>::max());
        
        // 割当
        for (const auto& main_building : this->main_buildings) {
            if (reachable_vertices.find(main_building) != reachable_vertices.end()) {
                this->facility_assignment_to_main_building[main_building] = facility;
            }
        }

    }

}

void Net_FSLP::assign_main_building_to_demand() {
    // 主要建物の需要点に対する割当＝最寄り
    std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->main_building_shortest_path_tree;

    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;

    for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
        Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
        if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_MAIN_BUILDINGS) {
            assignment_tree_r[parent].push_back(i);
        }
    }

    // 主要建物を需要点に割当
    for (const auto& main_building: this->main_buildings) {
        // 主要建物から最も近い頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> reachable_vertices;
        reachable_vertices.insert(main_building);
        this->net_ptr->search_reachable_vertices(main_building, 
                                                 assignment_tree, 
                                                 assignment_tree_r,
                                                 reachable_vertices, 
                                                 std::numeric_limits<double>::max());
        
        // 割当
        for (const auto& demand : this->demands) {
            if (reachable_vertices.find(demand) != reachable_vertices.end()) {
                this->main_building_assignment_to_demand[demand] = main_building;
            }
        }

    }
}

void Net_FSLP::update_facility_assignment_to_demand() {
    std::pair<bool, size_t> assigned_facility;
    std::pair<bool, size_t> assigned_sign;
    std::pair<bool, size_t> assigned_facility_to_sign;
    for (const auto& demand : this->demands) {
        assigned_facility = this->get_assigned_facility_to_demand(demand);
        if (!assigned_facility.first) {
            // サービス供給点が割り当てられていない場合
            assigned_sign = this->get_assigned_sign_to_demand(demand);
            if (assigned_sign.first) {
                // サインが割り当てられている場合
                // サインに割り当てられているサービス供給点を需要点に割当
                assigned_facility_to_sign = this->get_assigned_facility_to_sign(assigned_sign.second);

                if (!assigned_facility_to_sign.first) {
                    throw std::runtime_error(
                        "Sign has to have assigned facility.\n"
                        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                    );
                }

                this->facility_assignment_to_demand[demand] = assigned_facility_to_sign.second;

            }
        }
    }
}

//** Cost Function Methods **//
double Net_FSLP::calculate_cost(Net_2::vertex_descriptor demand) const {
    std::pair<bool, Net_2::vertex_descriptor> assigned_facility = this->get_assigned_facility_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_sign = this->get_assigned_sign_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_facility_to_sign = this->get_assigned_facility_to_sign(assigned_sign.second);
    std::pair<bool, Net_2::vertex_descriptor> assigned_main_building = this->get_assigned_main_building_to_demand(demand);

    double cost;
    double cost_demand_facility;
    double cost_demand_waystop;
    double cost_waystop_facility;
    double cost_demand_main_building;
    double cost_main_building_facility;

    if (assigned_facility.first) {
        // サービス供給点が割り当てられている場合
        return calculate_cost_to_nearest_facility_identified(demand);

    } else {
        // サービス供給点が割り当てられていない場合
        return calculate_cost_to_nearest_facility_unidentified(demand);

    }

    return cost;
}

double Net_FSLP::calculate_cost_to_nearest_facility_identified(Net_2::vertex_descriptor demand) const {

    double cost_demand_facility {0.0}; // 需要点 --> サービス供給点
    cost_demand_facility = this->facility_shortest_path_tree.at(demand).second;

    return 2 * cost_demand_facility; // 需要点 --> サービス供給点 --> 需要点

}

double Net_FSLP::calculate_cost_to_nearest_facility_unidentified(Net_2::vertex_descriptor demand) const {
    
    // いったん最寄りの主要施設に向かう
    if (!this->get_assigned_main_building_to_demand(demand).first) {

        throw std::runtime_error(
            "Demand has to have assigned main building.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );

    }

    std::pair<bool, Net_2::vertex_descriptor> waystop_existance = find_waystop(demand);
    if (waystop_existance.first) {
        return calculate_cost_via_waystop(demand, waystop_existance.second);
    } else {
        return calculate_cost_via_main_building(demand, this->get_assigned_main_building_to_demand(demand).second);
    }


}

std::pair<bool, Net_2::vertex_descriptor> Net_FSLP::find_waystop(Net_2::vertex_descriptor demand) const {

    std::deque<Net_2::vertex_descriptor> path_to_main_building = this->calculate_path_to_main_building(demand);
    std::pair<bool, Net_2::vertex_descriptor> waystop_existance = std::make_pair<bool, Net_2::vertex_descriptor>(false, 0);
    for (auto& v_on_the_way : path_to_main_building) {
        std::pair<bool, Net_2::vertex_descriptor> assigned_facility_to_waystop = this->get_assigned_facility_to_demand(v_on_the_way);
        if (assigned_facility_to_waystop.first) {
            // 途中で向かうべきサービス供給点が見つかった
            waystop_existance.first = true;
            waystop_existance.second = v_on_the_way;
            break;
        }
    }

    return waystop_existance;

}

double Net_FSLP::calculate_cost_via_waystop(Net_2::vertex_descriptor demand, Net_2::vertex_descriptor waystop) const {
    
    double cost_demand_facility {0.0}; // 需要点 --> サービス供給点
    double cost_demand_waystop {0.0}; // 需要点 --> 中継地点
    double cost_waystop_facility {0.0}; // 中継地点 --> サービス供給点

    cost_demand_waystop = this->main_building_shortest_path_tree.at(demand).second - this->main_building_shortest_path_tree.at(waystop).second;
    cost_waystop_facility = this->facility_shortest_path_tree.at(waystop).second;
    cost_demand_facility = this->facility_shortest_path_tree.at(demand).second;

    return cost_demand_waystop + cost_waystop_facility + cost_demand_facility; // 需要点 --> 中継地点 --> サービス供給点 --> 需要点

}

double Net_FSLP::calculate_cost_via_main_building(Net_2::vertex_descriptor demand, Net_2::vertex_descriptor assigned_main_building) const {
    
    double cost_demand_facility {0.0}; // 需要点 --> サービス供給点
    double cost_demand_main_building {0.0}; // 需要点 --> 主要施設
    double cost_main_building_facility {0.0}; // 主要施設 --> サービス供給点

    cost_demand_main_building = this->main_building_shortest_path_tree.at(demand).second;
    cost_main_building_facility = this->facility_shortest_path_tree.at(assigned_main_building).second;
    cost_demand_facility = this->facility_shortest_path_tree.at(demand).second;

    return cost_demand_main_building + cost_main_building_facility + cost_demand_facility; // 需要点 --> 主要施設 --> サービス供給点 --> 需要点

}