// include header
#include "domain/flp/net_sgflp.h"

// include stl util
#include "core/util/std_vector_util.h"
#include "core/util/random_engine.h"

//** Static Member Variable **//
static std::uniform_real_distribution<double> dist(0.0, 1.0);
const Point_2 Net_SGFLP::DUMMY_POINT_FACILITIES = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));
const Point_2 Net_SGFLP::DUMMY_POINT_SIGNS = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));
const Point_2 Net_SGFLP::DUMMY_POINT_ANCHORS = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));
const size_t Net_SGFLP::COST_PATTERN_DinA = 0;
const size_t Net_SGFLP::COST_PATTERN_DinF = 1;
const size_t Net_SGFLP::COST_PATTERN_DinS = 2;
const size_t Net_SGFLP::COST_PATTERN_DoutA = 3;
const size_t Net_SGFLP::COST_PATTERN_DoutF = 4;
const size_t Net_SGFLP::COST_PATTERN_DoutS = 5;
const size_t Net_SGFLP::COST_PATTERN_Duncovered = 6;

//** Constructor **//
Net_SGFLP::Net_SGFLP() : net_ptr(std::make_shared<Net_2>()) {} 
Net_SGFLP::Net_SGFLP(std::shared_ptr<Net_2> net_ptr) : net_ptr(net_ptr) {
    if (!this->net_ptr) {
        throw std::runtime_error("net_ptr is not initialized.\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

//** Getter **//
Point_2 Net_SGFLP::get_inner_point() const {
    return inner_point;
}
double Net_SGFLP::get_facility_visible_length() const {
    return facility_visible_length;
}
double Net_SGFLP::get_sign_visible_length() const {
    return sign_visible_length;
}
double Net_SGFLP::get_anchor_visible_length() const {
    return anchor_visible_length;
}
std::vector<Net_2::vertex_descriptor> Net_SGFLP::get_demands() const {
    return demands;
}
std::vector<Net_2::vertex_descriptor> Net_SGFLP::get_facilities() const {
    return  facilities;
}
std::vector<Net_2::vertex_descriptor> Net_SGFLP::get_signs() const {
    return signs;
}
std::vector<Net_2::vertex_descriptor> Net_SGFLP::get_anchors() const {
    return anchors;
}
Net_2::vertex_descriptor Net_SGFLP::get_dummy_vertex_facilities() const {
    return dummy_vertex_facilities;
}
Net_2::vertex_descriptor Net_SGFLP::get_dummy_vertex_signs() const {
    return dummy_vertex_signs;
}
Net_2::vertex_descriptor Net_SGFLP::get_dummy_vertex_anchors() const {
    return dummy_vertex_anchors;
}
std::unordered_map<Net_2::vertex_descriptor, 
                   std::vector<std::pair<Net_2::vertex_descriptor, double>>
> Net_SGFLP::get_facility_coverage_trees() const {
    return facility_coverage_trees;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_SGFLP::get_facility_shortest_path_tree() const {
    return facility_shortest_path_tree;
}
std::unordered_map<Net_2::vertex_descriptor, 
                   std::vector<std::pair<Net_2::vertex_descriptor, double>>
> Net_SGFLP::get_sign_coverage_trees() const {
    return sign_coverage_trees;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_SGFLP::get_sign_shortest_path_tree() const {
    return sign_shortest_path_tree;
}
std::unordered_map<Net_2::vertex_descriptor, 
                   std::vector<std::pair<Net_2::vertex_descriptor, double>>
> Net_SGFLP::get_anchor_coverage_trees() const {
    return anchor_coverage_trees;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_SGFLP::get_anchor_shortest_path_tree() const {
    return anchor_shortest_path_tree;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_SGFLP::get_facility_assignment_to_demand() const {
    return facility_assignment_to_demand;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_SGFLP::get_sign_assignment_to_demand() const {
    return sign_assignment_to_demand;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_SGFLP::get_anchor_assignment_to_demand() const {
    return anchor_assignment_to_demand;
}
std::vector<std::vector<Net_2::vertex_descriptor>> Net_SGFLP::get_entity_groups() const {
    return entity_groups;
}
std::vector<std::vector<double>> Net_SGFLP::get_visibility_distance_matrix() const {
    return visibility_distance_matrix;
}
std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> Net_SGFLP::get_navigation_assignment() const {
    return navigation_assignment;
}

//** Setter **//
void Net_SGFLP::set_inner_point(const Point_2 inner_point) {
    this->inner_point = inner_point;
}
void Net_SGFLP::set_facility_visible_length(const double visible_length) {
    this->facility_visible_length = visible_length;
}
void Net_SGFLP::set_sign_visible_length(const double visible_length) {
    this->sign_visible_length = visible_length;
}
void Net_SGFLP::set_anchor_visible_length(const double visible_length) {
    this->anchor_visible_length = visible_length;
}
void Net_SGFLP::set_demands() {
    std::unordered_set<Net_2::vertex_descriptor> demands_set;
    std::vector<Net_2::vertex_descriptor> prohibited_vertices={dummy_vertex_facilities, dummy_vertex_signs, dummy_vertex_anchors};
    demands_set = this->net_ptr->calculate_reachable_vertices(this->inner_point, prohibited_vertices);
    this->demands = std::vector<Net_2::vertex_descriptor>(demands_set.begin(), demands_set.end());
}
void Net_SGFLP::set_demands(const Point_2 inner_point) {
    set_inner_point(inner_point);
    set_demands();
}
void Net_SGFLP::set_demands(const std::vector<Net_2::vertex_descriptor> demands) {
    this->demands = demands;
}
void Net_SGFLP::set_facilities(const std::vector<Net_2::vertex_descriptor> facilities) {
    this->facilities = facilities;
}
void Net_SGFLP::set_signs(const std::vector<Net_2::vertex_descriptor> signs) {
    this->signs = signs;
}
void Net_SGFLP::set_anchors(const std::vector<Net_2::vertex_descriptor> anchors) {
    this->anchors = anchors;
}
void Net_SGFLP::set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities) {
    this->dummy_vertex_facilities = dummy_vertex_facilities;
}
void Net_SGFLP::set_dummy_vertex_signs(const Net_2::vertex_descriptor dummy_vertex_signs) {
    this->dummy_vertex_signs = dummy_vertex_signs;
}
void Net_SGFLP::set_dummy_vertex_anchors(const Net_2::vertex_descriptor dummy_vertex_anchors) {
    this->dummy_vertex_anchors = dummy_vertex_anchors;
}

//** Network Methods **//
void Net_SGFLP::build_trees() {
    build_facility_coverage_trees();
    build_facility_shortest_path_trees();
    build_facility_shortest_path_tree();
    
    build_sign_coverage_trees();
    build_sign_shortest_path_trees();
    build_sign_shortest_path_tree();
    
    build_anchor_coverage_trees();
    build_anchor_shortest_path_trees();
    build_anchor_shortest_path_tree();
}

void Net_SGFLP::clear_trees() {
    // facility_coverage_trees、sign_coverage_treesはキャッシュしておくため、clearしない
    // facility_shortest_path_trees、sign_shortest_path_treesはキャッシュしておくため、clearしない
    // anchorは不変のため、anchor_coverage_trees、anchor_shortest_path_trees、anchor_shortest_path_treeはclearしない
    facility_shortest_path_tree.clear();
    sign_shortest_path_tree.clear();
}

void Net_SGFLP::insert_dummy_vertex_facilities() {
    
    std::shared_ptr<Node_2> dummy_node_facilities_ptr = std::make_shared<Node_2>(DUMMY_POINT_FACILITIES, true);

    Net_2::vertex_descriptor dummy_vertex_facilities = boost::add_vertex(dummy_node_facilities_ptr, *(this->net_ptr));
    this->set_dummy_vertex_facilities(dummy_vertex_facilities);

    for (const auto& facility_vertex : this->facilities) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], (*(this->net_ptr))[facility_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_facilities, facility_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_SGFLP::insert_dummy_vertex_signs() {
    
    std::shared_ptr<Node_2> dummy_node_signs_ptr = std::make_shared<Node_2>(DUMMY_POINT_SIGNS, true);

    Net_2::vertex_descriptor dummy_vertex_signs = boost::add_vertex(dummy_node_signs_ptr, *(this->net_ptr));
    this->set_dummy_vertex_signs(dummy_vertex_signs);

    for (const auto& sign_vertex : this->signs) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_signs], (*(this->net_ptr))[sign_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_signs, sign_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_SGFLP::insert_dummy_vertex_anchors() {
    
    std::shared_ptr<Node_2> dummy_node_anchors_ptr = std::make_shared<Node_2>(DUMMY_POINT_ANCHORS, true);

    Net_2::vertex_descriptor dummy_vertex_anchors = boost::add_vertex(dummy_node_anchors_ptr, *(this->net_ptr));
    this->set_dummy_vertex_anchors(dummy_vertex_anchors);

    for (const auto& anchor_vertex : this->anchors) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_anchors], (*(this->net_ptr))[anchor_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_anchors, anchor_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_SGFLP::remove_dummy_vertices() {
    std::vector<Net_2::vertex_descriptor> to_remove = {
        dummy_vertex_facilities,
        dummy_vertex_signs,
        dummy_vertex_anchors
    };

    // 削除対象の頂点を大きい順にソート（削除時のインデックスずれを防ぐ）
    std::sort(to_remove.rbegin(), to_remove.rend());

    for (int v : to_remove) {
        if (v != UNINITIALIZED_DUMMY_VERTEX_FACILITIES && 
            v != UNINITIALIZED_DUMMY_VERTEX_SIGNS && 
            v != UNINITIALIZED_DUMMY_VERTEX_ANCHORS) {
            this->net_ptr->remove_vertex(v);
        }
    }

    // 無効な値にリセット
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;
    dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;
    dummy_vertex_anchors = UNINITIALIZED_DUMMY_VERTEX_ANCHORS;
}

void Net_SGFLP::remove_dummy_vertex_facilities() {
    boost::clear_vertex(dummy_vertex_facilities, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_facilities, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;
}

void Net_SGFLP::remove_dummy_vertex_signs() {
    boost::clear_vertex(dummy_vertex_signs, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_signs, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;
}

void Net_SGFLP::remove_dummy_vertex_anchors() {
    boost::clear_vertex(dummy_vertex_anchors, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_anchors, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_anchors = UNINITIALIZED_DUMMY_VERTEX_ANCHORS;
}

void Net_SGFLP::update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                              const Net_2::vertex_descriptor next_facility_vertex) 
{
    boost::remove_edge(dummy_vertex_facilities, prev_facility_vertex, *(this->net_ptr));

    std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], 
                                                                      (*(this->net_ptr))[next_facility_vertex]);
    dummy_edge_ptr->set_is_dummy(true);
    boost::add_edge(this->dummy_vertex_facilities, next_facility_vertex, dummy_edge_ptr, *(this->net_ptr));
}

void Net_SGFLP::update_dummy_vertex_signs(const Net_2::vertex_descriptor prev_sign_vertex, 
                                         const Net_2::vertex_descriptor next_sign_vertex) 
{
    boost::remove_edge(dummy_vertex_signs, prev_sign_vertex, *(this->net_ptr));

    std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_signs], 
                                                                      (*(this->net_ptr))[next_sign_vertex]);
    dummy_edge_ptr->set_is_dummy(true);
    boost::add_edge(this->dummy_vertex_signs, next_sign_vertex, dummy_edge_ptr, *(this->net_ptr));
}

void Net_SGFLP::build_facility_coverage_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録のサービス供給点
    for (const auto& facility : this->facilities) {
        if (this->facility_coverage_trees.find(facility) == this->facility_coverage_trees.end()) {       
            this->facility_coverage_trees[facility] = this->net_ptr->calculate_shortest_path_tree(facility, 
                                                                                                Net_2::MODE_VISIBILITY, 
                                                                                                false, 
                                                                                                false, 
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

void Net_SGFLP::build_facility_shortest_path_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録のサービス供給点
    for (const auto& facility : this->facilities) {
        if (this->facility_shortest_path_trees.find(facility) == this->facility_shortest_path_trees.end()) {       
            this->facility_shortest_path_trees[facility] = this->net_ptr->calculate_shortest_path_tree(facility, 
                                                                                                Net_2::MODE_ROUTE, 
                                                                                                true, 
                                                                                                true, 
                                                                                                prohibited_vertices);
        }
    }

    // サービス供給点でなくなったもの
    for (auto it = this->facility_shortest_path_trees.begin(); it != this->facility_shortest_path_trees.end(); ) {
        if (std::find(this->facilities.begin(), this->facilities.end(), it->first) == this->facilities.end()) {
            it = this->facility_shortest_path_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }

}

void Net_SGFLP::build_facility_shortest_path_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_signs, dummy_vertex_anchors};
    this->facility_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_facilities(), 
                                                                                    Net_2::MODE_ROUTE, 
                                                                                    true, 
                                                                                    true, 
                                                                                    prohibited_vertices);
}

void Net_SGFLP::build_sign_coverage_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録のサイン
    for (const auto& sign : this->signs) {
        if (this->sign_coverage_trees.find(sign) == this->sign_coverage_trees.end()) {       
            this->sign_coverage_trees[sign] = this->net_ptr->calculate_shortest_path_tree(sign, 
                                                                                       Net_2::MODE_VISIBILITY, 
                                                                                       false, 
                                                                                       false, 
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

void Net_SGFLP::build_sign_shortest_path_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録のサイン
    for (const auto& sign : this->signs) {
        if (this->sign_shortest_path_trees.find(sign) == this->sign_shortest_path_trees.end()) {       
            this->sign_shortest_path_trees[sign] = this->net_ptr->calculate_shortest_path_tree(sign, 
                                                                                       Net_2::MODE_ROUTE, 
                                                                                       true, 
                                                                                       true, 
                                                                                       prohibited_vertices);
        }
    }

    // サインでなくなったもの
    for (auto it = this->sign_shortest_path_trees.begin(); it != this->sign_shortest_path_trees.end(); ) {
        if (std::find(this->signs.begin(), this->signs.end(), it->first) == this->signs.end()) {
            it = this->sign_shortest_path_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }
    
}

void Net_SGFLP::build_sign_shortest_path_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, dummy_vertex_anchors};
    this->sign_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_signs(), 
                                                                                         Net_2::MODE_ROUTE, 
                                                                                         true, 
                                                                                         true, 
                                                                                         prohibited_vertices);
}

void Net_SGFLP::build_anchor_coverage_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録の拠点
    for (const auto& anchor : this->anchors) {
        if (this->anchor_coverage_trees.find(anchor) == this->anchor_coverage_trees.end()) {       
            this->anchor_coverage_trees[anchor] = this->net_ptr->calculate_shortest_path_tree(anchor, 
                                                                                       Net_2::MODE_VISIBILITY, 
                                                                                       false, 
                                                                                       false, 
                                                                                       prohibited_vertices);
        }
    }

    // 拠点でなくなったもの
    for (auto it = this->anchor_coverage_trees.begin(); it != this->anchor_coverage_trees.end(); ) {
        if (std::find(this->anchors.begin(), this->anchors.end(), it->first) == this->anchors.end()) {
            it = this->anchor_coverage_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }
    
}

void Net_SGFLP::build_anchor_shortest_path_trees() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                               dummy_vertex_signs, 
                                                               dummy_vertex_anchors};
    
    // 未登録の拠点
    for (const auto& anchor : this->anchors) {
        if (this->anchor_shortest_path_trees.find(anchor) == this->anchor_shortest_path_trees.end()) {       
            this->anchor_shortest_path_trees[anchor] = this->net_ptr->calculate_shortest_path_tree(anchor, 
                                                                                       Net_2::MODE_ROUTE, 
                                                                                       true, 
                                                                                       true, 
                                                                                       prohibited_vertices);
        }
    }

    // 拠点でなくなったもの
    for (auto it = this->anchor_shortest_path_trees.begin(); it != this->anchor_shortest_path_trees.end(); ) {
        if (std::find(this->anchors.begin(), this->anchors.end(), it->first) == this->anchors.end()) {
            it = this->anchor_shortest_path_trees.erase(it); // 削除
        } else {
            ++it;
        }
    }
    
}

void Net_SGFLP::build_anchor_shortest_path_tree() {
    std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, dummy_vertex_signs};
    this->anchor_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_anchors(), 
                                                                                         Net_2::MODE_ROUTE, 
                                                                                         true, 
                                                                                         true, 
                                                                                         prohibited_vertices);
}

std::deque<Net_2::vertex_descriptor> Net_SGFLP::calculate_path_to_anchor(Net_2::vertex_descriptor demand) const {
    std::deque<Net_2::vertex_descriptor> path_to_anchor;
    std::pair<bool, Net_2::vertex_descriptor> anchor = this->get_assigned_anchor_to_demand(demand);
    
    if (!anchor.first) {
        throw std::runtime_error(
            "Demand has to have assigned main building.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
    for (auto& vertex=demand; vertex != anchor.second; vertex = this->anchor_shortest_path_tree.at(vertex).first) {
        path_to_anchor.push_back(vertex);
    }

    path_to_anchor.push_back(anchor.second);
    
    return path_to_anchor;
}

//** Initialization Methods **//
Net_2::vertex_descriptor Net_SGFLP::search_nearest_demand(const Point_2 point) const {
    Net_2::vertex_descriptor nearest_vertex = net_ptr->find_nearest_node(point);

    if (find_index(demands, nearest_vertex) != -1) {
        // pに最近隣の頂点が需要点の中にある
        return nearest_vertex;
    } else {
        // pに最近隣の頂点が需要点の中にない
        // 最近隣の頂点から順に需要点を探索する
        std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities, 
                                                                   dummy_vertex_signs, 
                                                                   dummy_vertex_anchors};
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

std::vector<Net_2::vertex_descriptor> Net_SGFLP::search_nearest_demands(const std::vector<Point_2> points) const {
    std::vector<Net_2::vertex_descriptor> nearest_demands {};
    for (const auto& p : points) {
        nearest_demands.push_back(search_nearest_demand(p));
    }

    return nearest_demands;
    
}

std::vector<Net_2::vertex_descriptor> Net_SGFLP::select_demands_at_random(const size_t selected_num) {
    // 重複無しの無作為抽出
    return random_sampling(demands, selected_num, false);
}

void Net_SGFLP::initialize_facilities(const std::vector<Point_2> facility_points) {

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

void Net_SGFLP::initialize_facilities(const size_t facility_num) {

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

void Net_SGFLP::initialize_signs(const std::vector<Point_2> sign_points) {

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

void Net_SGFLP::initialize_signs(const size_t sign_num) {

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

void Net_SGFLP::initialize_anchors(const std::vector<Point_2> anchor_points) {

    this->anchors = search_nearest_demands(anchor_points);

    if (this->dummy_vertex_anchors == UNINITIALIZED_DUMMY_VERTEX_ANCHORS) {
        this->insert_dummy_vertex_anchors();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing main buildings.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

}

void Net_SGFLP::initialize_entity_groups() {
    this->entity_groups.clear();

    std::vector<std::vector<Net_2::vertex_descriptor>> initial_entity_groups = {
        facilities,
        signs,
        anchors
    };

    this->entity_groups = initial_entity_groups;
}

void Net_SGFLP::initialize_visibility_distance_matrix() {
    this->visibility_distance_matrix.clear();

    if (this->entity_groups.size() != 3) {
        throw std::runtime_error(
            "Entity groups have to be initialized as {facilities, signs, anchors}.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    const double INF = std::numeric_limits<double>::max();

    std::vector<Net_2::vertex_descriptor> entities = {};
    entities.insert(entities.end(), entity_groups[0].begin(), entity_groups[0].end()); // サービス供給点
    entities.insert(entities.end(), entity_groups[1].begin(), entity_groups[1].end()); // サイン
    entities.insert(entities.end(), entity_groups[2].begin(), entity_groups[2].end()); // 拠点

    const std::size_t facility_end = this->facilities.size();
    const std::size_t sign_end = facility_end + this->signs.size();
    const std::size_t anchor_end = sign_end + this->anchors.size();

    const auto is_facility_or_anchor_index = [&](const std::size_t idx) {
        return idx < facility_end || (idx >= sign_end && idx < anchor_end);
    };

    // 互いに見えないものとして初期化
    std::vector<std::vector<double>> initial_visibility_distance_matrix {
        entities.size(),
        std::vector<double>(entities.size(), INF)
    };

    const auto get_coverage_tree = [&](const Net_2::vertex_descriptor entity)
        -> std::vector<std::pair<Net_2::vertex_descriptor, double>> {
        if (this->facility_coverage_trees.find(entity) != this->facility_coverage_trees.end()) {
            return this->facility_coverage_trees.at(entity); // サービス供給点
        }
        if (this->sign_coverage_trees.find(entity) != this->sign_coverage_trees.end()) {
            return this->sign_coverage_trees.at(entity); // サイン
        }
        if (this->anchor_coverage_trees.find(entity) != this->anchor_coverage_trees.end()) {
            return this->anchor_coverage_trees.at(entity); // 拠点
        }

        throw std::runtime_error(
            "Vantage entity has to be facility, sign or anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    };

    // 可視距離の計算
    for (std::size_t i {0}; i < entities.size(); ++i) {
        const Net_2::vertex_descriptor vantage_entity = entities.at(i);
        const std::vector<std::pair<Net_2::vertex_descriptor, double>> vantage_coverage_tree =
            get_coverage_tree(vantage_entity);

        //TODO begin here ========================================
        //TODO Net_2::calculate_visible_vertices と同じことをしているので、共通化できないか検討する
        // 最短経路木を反転する
        // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
        std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> vantage_coverage_tree_r;

        for (std::size_t k {0}; k < vantage_coverage_tree.size(); ++k) {
            const Net_2::vertex_descriptor parent = vantage_coverage_tree.at(k).first;
            if (parent != k && parent != Net_2::null_vertex()) {
                vantage_coverage_tree_r[parent].push_back(k);
            }
        }

        // 可視である頂点を探索する
        std::unordered_set<Net_2::vertex_descriptor> visible_vertices;

        visible_vertices.insert(vantage_entity);
        this->net_ptr->search_visible_vertices(vantage_entity, 
                                            vantage_coverage_tree, 
                                            vantage_coverage_tree_r,
                                            visible_vertices, 
                                            std::numeric_limits<double>::max()); // 経路割当の際は可視距離の制限なし
        //TODO end here ========================================
        
        for (std::size_t j {i}; j < entities.size(); ++j) {
            if (i == j) {
                // 同一の点は可視距離0
                initial_visibility_distance_matrix.at(i).at(j) = 0.0;
            } else {
                Net_2::vertex_descriptor target_entity = entities.at(j);

                if (visible_vertices.find(target_entity) != visible_vertices.end()) {
                    // 可視である場合は距離を計算
                    double vis_distance = vantage_coverage_tree.at(target_entity).second; // vantage_entityからtarget_entityへの可視距離
                    initial_visibility_distance_matrix.at(i).at(j) = vis_distance;
                    initial_visibility_distance_matrix.at(j).at(i) = vis_distance;
                }
            }
        }

    }

    // facilitiesとanchorを結ぶダミーノードを追加
    for (std::size_t i {0}; i < initial_visibility_distance_matrix.size(); ++i) {
        initial_visibility_distance_matrix.at(i).push_back(
            is_facility_or_anchor_index(i) ? 0.0 : INF
        );
    }

    std::vector<double> last_row(entities.size() + 1, INF);
    for (std::size_t i {0}; i < entities.size(); ++i) {
        if (is_facility_or_anchor_index(i)) {
            last_row.at(i) = 0.0;
        }
    }
    last_row.back() = 0.0;

    initial_visibility_distance_matrix.push_back(last_row);

    this->visibility_distance_matrix = initial_visibility_distance_matrix;

}

void Net_SGFLP::initialize_navigation_assignment() {
    std::cout << "Initializing navigation assignment..." << std::endl;
    this->navigation_assignment.clear();

    this->initialize_entity_groups();
    this->initialize_visibility_distance_matrix();

    this->assign_navigation();
    
}

//** Clear Methods **//
void Net_SGFLP::clear() {
    remove_dummy_vertices();
}

//** Constraint Method **//
std::pair<bool, Net_2::vertex_descriptor> Net_SGFLP::get_assigned_facility_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->facility_assignment_to_demand.find(demand) != this->facility_assignment_to_demand.end()) {
        return std::make_pair(true, this->facility_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_SGFLP::get_assigned_sign_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->sign_assignment_to_demand.find(demand) != this->sign_assignment_to_demand.end()) {
        return std::make_pair(true, this->sign_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_SGFLP::get_assigned_anchor_to_demand(Net_2::vertex_descriptor demand) const {
    if (this->anchor_assignment_to_demand.find(demand) != this->anchor_assignment_to_demand.end()) {
        return std::make_pair(true, this->anchor_assignment_to_demand.at(demand));
    } else {
        return std::make_pair(false, 0);
    }
}

std::pair<bool, Net_2::vertex_descriptor> Net_SGFLP::get_assigned_entity_on_navigation(Net_2::vertex_descriptor entity) const {
    if (this->navigation_assignment.find(entity) != this->navigation_assignment.end()) {
        return std::make_pair(true, this->navigation_assignment.at(entity));
    } else {
        return std::make_pair(false, 0);
    }
}

void Net_SGFLP::build_assignments() {
    assign_facility_to_demand();
    assign_sign_to_demand();
    assign_anchor_to_demand();

    if (this->entity_groups.empty()) {
        initialize_navigation_assignment();
    } else {
        update_navigation_assignment();
    }

}

void Net_SGFLP::clear_assignments() {
    facility_assignment_to_demand.clear();
    sign_assignment_to_demand.clear();
    anchor_assignment_to_demand.clear();

    navigation_assignment.clear();
}

void Net_SGFLP::assign_facility_to_demand() {
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
                                                this->facility_visible_length);
        
        // 割当
        for (const auto& visible_vertex : visible_vertices) {
            if (find_index(this->demands, visible_vertex) != -1) {
                // 可視である需要点
                double vis_len = assignment_tree[visible_vertex].second; // 可視長さを取得
                bool update = false;
                if (this->facility_assignment_to_demand.find(visible_vertex) == this->facility_assignment_to_demand.end()) {
                    // 未割当
                    update = true;
                } else if (vis_min_lens.find(visible_vertex) == vis_min_lens.end()) {
                    // 可視長さの記録がない（初めて可視になる）
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

void Net_SGFLP::assign_sign_to_demand() {
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
                                               this->sign_visible_length);
        
        // 割当
        for (const auto& visible_vertex : visible_vertices) {
            if (find_index(this->demands, visible_vertex) != -1) {
                // 可視である需要点
                double vis_len = assignment_tree[visible_vertex].second; // 可視長さを取得
                bool update = false;
                if (this->sign_assignment_to_demand.find(visible_vertex) == this->sign_assignment_to_demand.end()) {
                    // 未割当
                    update = true;
                } else if (vis_min_lens.find(visible_vertex) == vis_min_lens.end()) {
                    // 可視長さの記録がない（初めて可視になる）
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

void Net_SGFLP::assign_anchor_to_demand() {
    // 拠点の需要点に対する割当＝可視かつ最寄り
    std::unordered_map<Net_2::vertex_descriptor, double> vis_min_lens; // サービス供給点までの可視長さ
    for (const auto& anchor : this->anchors) {
        std::vector<std::pair<Net_2::vertex_descriptor, double>> assignment_tree = this->anchor_coverage_trees[anchor];
    
        // 最短経路木を反転する
        // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
        std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> assignment_tree_r;
    
        for (std::size_t i {0}; i < assignment_tree.size(); ++i) {
            Net_2::vertex_descriptor parent = assignment_tree.at(i).first;
            if (parent != i && parent != UNINITIALIZED_DUMMY_VERTEX_SIGNS) {
                assignment_tree_r[parent].push_back(i);
            }
        }
    
        // 拠点を需要点に割当
        // 拠点から見える頂点を探索
        std::unordered_set<Net_2::vertex_descriptor> visible_vertices;
        visible_vertices.insert(anchor);
        this->net_ptr->search_visible_vertices(anchor, 
                                               assignment_tree, 
                                               assignment_tree_r,
                                               visible_vertices, 
                                               this->anchor_visible_length);
        
        // 割当
        for (const auto& visible_vertex : visible_vertices) {
            if (find_index(this->demands, visible_vertex) != -1) {
                // 可視である需要点
                double vis_len = assignment_tree[visible_vertex].second; // 可視長さを取得
                bool update = false;
                if (this->anchor_assignment_to_demand.find(visible_vertex) == this->anchor_assignment_to_demand.end()) {
                    // 未割当
                    update = true;
                } else if (vis_min_lens.find(visible_vertex) == vis_min_lens.end()) {
                    // 可視長さの記録がない（初めて可視になる）
                    update = true;
                } else if (vis_len < vis_min_lens[visible_vertex]) {
                    // 可視長さが短くなる
                    update = true;
                }

                if (update) {
                    this->anchor_assignment_to_demand[visible_vertex] = anchor;
                    vis_min_lens[visible_vertex] = vis_len;
                }
                
            }
        }

    }
}

void Net_SGFLP::assign_navigation() {
    std::vector<Net_2::vertex_descriptor> entities {};
    entities.insert(entities.end(), entity_groups[0].begin(), entity_groups[0].end()); // サービス供給点
    entities.insert(entities.end(), entity_groups[1].begin(), entity_groups[1].end()); // サイン
    entities.insert(entities.end(), entity_groups[2].begin(), entity_groups[2].end()); // 拠点

    if (this->visibility_distance_matrix.empty()) {
        throw std::runtime_error(
            "Visibility distance matrix has not been initialized.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    if (this->visibility_distance_matrix.size() != entities.size() + 1) {
        throw std::runtime_error(
            "Visibility distance matrix size does not match entities + dummy node.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    const std::size_t node_count = this->visibility_distance_matrix.size();
    const std::size_t dummy_node_idx = node_count - 1;
    const double INF = std::numeric_limits<double>::max();

    // ダミーノードを根とする最短経路木を構築（行列に対するDijkstra）
    std::vector<double> min_dists(node_count, INF);
    std::vector<std::size_t> parents(node_count, dummy_node_idx);
    std::vector<bool> is_fixed(node_count, false);

    min_dists[dummy_node_idx] = 0.0;
    parents[dummy_node_idx] = dummy_node_idx;

    for (std::size_t step = 0; step < node_count; ++step) {
        std::size_t current = dummy_node_idx;
        double current_dist = INF;

        // 未確定ノードのなかで最小の距離のノードを選択
        for (std::size_t i = 0; i < node_count; ++i) {
            if (!is_fixed[i] && min_dists[i] < current_dist) {
                current_dist = min_dists[i];
                current = i;
            }
        }

        if (current_dist == INF) {
            // これ以上更新できない
            break;
        }

        // 選択されたノードを確定させる
        is_fixed[current] = true;

        // currentから行列に基づいて隣接ノードへの距離を更新する
        const auto& row = this->visibility_distance_matrix.at(current);
        
        if (row.size() != node_count) {
            throw std::runtime_error(
                "Visibility distance matrix has to be square.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }

        for (std::size_t next = 0; next < node_count; ++next) {
            if (is_fixed[next]) {
                continue;
            }

            const double edge_weight = row.at(next);
            if (edge_weight == INF) {
                continue;
            }

            const double candidate_dist = min_dists[current] + edge_weight;
            if (candidate_dist < min_dists[next]) {
                // 更新値が現在値より小さければ更新
                min_dists[next] = candidate_dist;
                parents[next] = current;
            }
        }
    }

    // 最短経路木の親に従って、サインの遷移先を割り当てる
    const std::size_t sign_begin = this->facilities.size();
    const std::size_t sign_end = sign_begin + this->signs.size();
    for (std::size_t sign_idx = sign_begin; sign_idx < sign_end; ++sign_idx) {
        
        if (sign_idx >= entities.size()) {
            throw std::runtime_error(
                "Sign index is out of entities range.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }

        if (parents[sign_idx] >= entities.size()) {
            throw std::runtime_error(
                "Parent index of sign points to dummy or invalid node.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }

        this->navigation_assignment[entities[sign_idx]] = entities[parents[sign_idx]];
    }
}

std::pair<size_t, size_t> Net_SGFLP::update_entity_groups() {
    std::vector<std::pair<size_t, size_t>> update_info;
    
    // 更新後のエンティティ
    std::vector<std::vector<Net_2::vertex_descriptor>> updated_entity_groups = {
        facilities,
        signs,
        anchors
    };

    // 更新後と前のエンティティを比較
    //!更新されなかったエンティティのインデックスは不変であることが前提
    for (size_t group_idx = 0; group_idx < updated_entity_groups.size(); ++group_idx) {
        const auto& group = updated_entity_groups[group_idx];
        for (size_t vertex_idx = 0; vertex_idx < group.size(); ++vertex_idx) {
            if (group[vertex_idx] != this->entity_groups[group_idx][vertex_idx]) {
                // 更新後と前で頂点が異なる場合、更新情報に追加

                if (group_idx == 2) {
                    // 拠点は更新されないためエラー
                    throw std::runtime_error(
                        "Anchor should not be updated.\n"
                        "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                    );
                }
                
                update_info.emplace_back(group_idx, vertex_idx);
            }
        }
    }
   
    // 更新の整合性チェック
    if (update_info.empty()) {
        std::cout << "Warning: No entity has been updated." << std::endl;
        return std::make_pair(-1, -1);
    }

    if (update_info.size() > 1) {
        std::cout << "Warning: Multiple entities have been updated. Rebuilding navigation assignment from scratch." << std::endl;
        for (const auto& info : update_info) {
            Net_2::vertex_descriptor updated_vertex = updated_entity_groups[info.first][info.second];
            std::cout << "Updated entity group index: " << info.first << ", vertex index: " << info.second << ", vertex: " << updated_vertex << std::endl;
        }
        this->entity_groups = updated_entity_groups;
        return std::make_pair(-2, -2);
    }

    // エンティティの更新
    this->entity_groups = updated_entity_groups;

    // 更新情報を返す
    return update_info.front();

}

void Net_SGFLP::update_visibility_distance_matrix() {
    // 更新されたエンティティを同定
    std::pair<size_t, size_t> update_info = update_entity_groups();
    if (update_info.first == -1 && update_info.second == -1) {
        // 更新されたエンティティがない場合は何もしない
        return;
    }
    if (update_info.first == -2 && update_info.second == -2) {
        // 複数エンティティが更新された場合は行列を全再構築する
        this->initialize_entity_groups();
        this->initialize_visibility_distance_matrix();
        return;
    }

    // 更新されたエンティティからの可視距離を計算
    Net_2::vertex_descriptor entity = this->entity_groups[update_info.first][update_info.second];
    std::vector<std::pair<Net_2::vertex_descriptor, double>> entity_coverage_tree;

    if (update_info.first == 0) {
        // サービス供給点
        entity_coverage_tree = this->facility_coverage_trees[entity];
    } else if (update_info.first == 1) {
        // サイン
        entity_coverage_tree = this->sign_coverage_trees[entity];
    } else if (update_info.first == 2) {
        // 拠点
        entity_coverage_tree = this->anchor_coverage_trees[entity];
    }

    //TODO begin here ========================================
    //TODO Net_2::calculate_visible_vertices と同じことをしているので、共通化できないか検討する
    // 最短経路木を反転する
    // 最短経路木上で，ある頂点に対して次に向かうべき頂点がわかるようにする
    std::unordered_map<Net_2::vertex_descriptor, std::vector<Net_2::vertex_descriptor>> entity_coverage_tree_r;

    for (std::size_t i {0}; i < entity_coverage_tree.size(); ++i) {
        Net_2::vertex_descriptor parent = entity_coverage_tree.at(i).first;
        if (parent != i && parent != Net_2::null_vertex()) {
            entity_coverage_tree_r[parent].push_back(i);
        }
    }

    // 可視である頂点を探索する
    std::unordered_set<Net_2::vertex_descriptor> visible_vertices;

    visible_vertices.insert(entity);
    this->net_ptr->search_visible_vertices(entity, 
                                           entity_coverage_tree, 
                                           entity_coverage_tree_r,
                                           visible_vertices, 
                                           std::numeric_limits<double>::max()); // 経路割当の際は可視距離の制限なし
    //TODO end here ========================================
    
    // 可視距離行列を更新する
    std::vector<Net_2::vertex_descriptor> entities {};
    entities.insert(entities.end(), entity_groups[0].begin(), entity_groups[0].end()); // サービス供給点
    entities.insert(entities.end(), entity_groups[1].begin(), entity_groups[1].end()); // サイン
    entities.insert(entities.end(), entity_groups[2].begin(), entity_groups[2].end()); // 拠点

    size_t entity_idx = find_index(entities, entity);
    
    if (entity_idx == -1) {
        throw std::runtime_error(
            "Updated entity is not found in entities list.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    //!ダミーノード（最終行、列）の値は変えない。サービス供給点、サイン、拠点の数は不変。
    for (size_t i {0}; i < entities.size(); ++i) {
        if (visible_vertices.find(entities.at(i)) != visible_vertices.end()) {
            // 可視である場合は距離を計算
            double vis_distance = entity_coverage_tree.at(i).second; // vantage_entityからtarget_entityへの可視距離
            this->visibility_distance_matrix.at(entity_idx).at(i) = vis_distance;
            this->visibility_distance_matrix.at(i).at(entity_idx) = vis_distance;
        } else {
            // 可視でない場合は距離を無限大にする
            this->visibility_distance_matrix.at(entity_idx).at(i) = std::numeric_limits<double>::max();
            this->visibility_distance_matrix.at(i).at(entity_idx) = std::numeric_limits<double>::max();
        }
    }

}

void Net_SGFLP::update_navigation_assignment() {
    // std::cout << "Updating navigation assignment..." << std::endl;
    this->navigation_assignment.clear();

    this->update_visibility_distance_matrix();
    this->assign_navigation();

}

//** Cost Function Methods **//
std::pair<size_t, double> Net_SGFLP::calculate_cost(Net_2::vertex_descriptor demand) const {
    std::pair<bool, Net_2::vertex_descriptor> assigned_facility = this->get_assigned_facility_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_sign = this->get_assigned_sign_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_anchor = this->get_assigned_anchor_to_demand(demand);

    if (assigned_anchor.first) {
        // 拠点が割り当てられている場合
        return calculate_cost_to_visible_anchor(demand, assigned_anchor.second);
    } else if (assigned_facility.first) {
        // サービス供給点が割り当てられている場合
        return calculate_cost_to_visible_facility(demand, assigned_facility.second);
    } else if (assigned_sign.first) {
        // サインが割り当てられている場合
        return calculate_cost_to_follow_signage(demand, assigned_sign.second);
    } else {
        // 何も割り当てられていない場合
        return calculate_cost_from_uncovered_demand(demand);
    }

}

std::pair<size_t, double> Net_SGFLP::calculate_cost_to_visible_facility(Net_2::vertex_descriptor demand, 
                                                                        Net_2::vertex_descriptor facility) const {

    // std::cout << "Calculating cost from demand " << demand << " to visible facility " << facility << std::endl;
    double cost_demand_facility {0.0}; // 需要点 --> サービス供給点
    cost_demand_facility = this->facility_coverage_trees.at(facility).at(demand).second;

    return std::make_pair(COST_PATTERN_DinF, cost_demand_facility);

}

std::pair<size_t, double> Net_SGFLP::calculate_cost_to_follow_signage(Net_2::vertex_descriptor demand, 
                                                                      Net_2::vertex_descriptor sign) const {

    // std::cout << "Calculating cost from demand " << demand << " to follow signage " << sign << std::endl;
    
    // std::cout 
    // << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10)
    // << "\tStart from demand " << std::to_string(demand) << " (" << (*(this->net_ptr))[demand]->x() << ", " << (*(this->net_ptr))[demand]->y() << ")" << std::endl;

    double cost {0.0}; // 需要点 --> サイン ... サイン --> サービス供給 or 拠点
    Net_2::vertex_descriptor curr_vertex = demand;
    Net_2::vertex_descriptor next_entity = this->navigation_assignment.at(sign);
    Net_2::vertex_descriptor last_entity;
    while (true) {
        // 次のエンティティへ向かう
        const std::vector<std::pair<std::size_t, double>>* shortest_path_tree = nullptr;
        if (find_index(this->anchors, next_entity) != -1) {
            shortest_path_tree = &this->anchor_shortest_path_trees.at(next_entity);
        } else if (find_index(this->signs, next_entity) != -1) {
            shortest_path_tree = &this->sign_shortest_path_trees.at(next_entity);
        } else if (find_index(this->facilities, next_entity) != -1) {
            shortest_path_tree = &this->facility_shortest_path_trees.at(next_entity);
        } else {
            throw std::runtime_error(
                "Next entity in navigation assignment has to be facility, sign or anchor.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }

        std::deque<std::pair<Net_2::vertex_descriptor, double>> path_r = this->net_ptr->calculate_shortest_path(next_entity, 
                                                                                                                curr_vertex, 
                                                                                                                *shortest_path_tree);
        
        if (path_r.empty()) {
            throw std::runtime_error(
                "No path from demand to sign.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );  
        }

        // curr_vertex --> next_entity
        std::deque<std::pair<Net_2::vertex_descriptor, double>> path = path_r; // 経路を反転させる
        std::reverse(path.begin(), path.end());

        std::pair<bool, Net_2::vertex_descriptor> waystop_existance = find_waystop(next_entity, path);
        if (waystop_existance.first) {
            // 中継地点がある場合は、そこまで向かう
            cost += calculate_cost_on_path(curr_vertex, 
                                           waystop_existance.second, 
                                           path);
            curr_vertex = waystop_existance.second;
        } else {
            // 次のエンティティまで向かう
            cost += path.front().second; // curr_vertexからnext_entityへの最短距離
            curr_vertex = next_entity;
        }

        // std::cout 
        // << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10)
        // << "\tArrived at " << std::to_string(curr_vertex) << " (" << (*(this->net_ptr))[curr_vertex]->x() << ", " << (*(this->net_ptr))[curr_vertex]->y() << ")" << std::endl;

        // 次のエンティティを更新
        if (this->navigation_assignment.find(next_entity) == this->navigation_assignment.end()) {
            // next_entityが見つからない場合はループを抜ける
            
            // next_entityがない場合はサービス供給点、拠点のいずれかであるはずなので、どちらにも見つからない場合はエラー
            if (find_index(this->facilities, next_entity) == -1 && 
                find_index(this->anchors, next_entity) == -1) {
                throw std::runtime_error(
                    "Last entity in navigation assignment is not found in facilities or anchors.\n"
                    "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                );
            }

            last_entity = next_entity;
            break;
        }

        next_entity = this->navigation_assignment.at(next_entity);
        
    }

    // 現在地から最後のエンティティまでの距離を加算する
    // std::cout
    // << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10)
    // << "\tEnd at " << std::to_string(last_entity) << " (" << (*(this->net_ptr))[last_entity]->x() << ", " << (*(this->net_ptr))[last_entity]->y() << ")" << std::endl;

    const std::vector<std::pair<std::size_t, double>>* shortest_path_tree = nullptr;
    if (find_index(this->anchors, last_entity) != -1) {
        shortest_path_tree = &this->anchor_shortest_path_trees.at(last_entity);
    } else if (find_index(this->signs, last_entity) != -1) {
        shortest_path_tree = &this->sign_shortest_path_trees.at(last_entity);
    } else if (find_index(this->facilities, last_entity) != -1) {
        shortest_path_tree = &this->facility_shortest_path_trees.at(last_entity);
    } else {
        throw std::runtime_error(
            "Next entity in navigation assignment has to be facility, sign or anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    std::deque<std::pair<Net_2::vertex_descriptor, double>> path_r = this->net_ptr->calculate_shortest_path(last_entity, 
                                                                                                            curr_vertex, 
                                                                                                            *shortest_path_tree);
    // curr_vertex --> last_entity
    std::deque<std::pair<Net_2::vertex_descriptor, double>> path = path_r; // 経路を反転させる
    std::reverse(path.begin(), path.end());
    
    cost += path.front().second; // curr_vertexからlast_entityへの最短距離

    return std::make_pair(COST_PATTERN_DinS, cost);

}

std::pair<size_t, double> Net_SGFLP::calculate_cost_to_visible_anchor(Net_2::vertex_descriptor demand, 
                                                                      Net_2::vertex_descriptor anchor) const {

    // std::cout << "Calculating cost from demand " << demand << " to visible anchor " << anchor << std::endl;
    double cost_demand_anchor {0.0}; // 需要点 --> 拠点
    cost_demand_anchor = this->anchor_coverage_trees.at(anchor).at(demand).second;

    return std::make_pair(COST_PATTERN_DinA, cost_demand_anchor);

}

std::pair<size_t, double> Net_SGFLP::calculate_cost_from_uncovered_demand(Net_2::vertex_descriptor demand) const {

    // std::cout 
    // << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10) 
    // << "Calculating cost from uncovered demand " << demand 
    // << " (" << (*(this->net_ptr))[demand]->x() << ", " << (*(this->net_ptr))[demand]->y() << ")" << std::endl;
    
    double cost {0.0};

    // まず最初に、最寄りの拠点に向かう
    const std::vector<std::pair<Net_2::vertex_descriptor, double>>* assignment_tree = &this->anchor_shortest_path_tree;
    std::deque<std::pair<Net_2::vertex_descriptor, double>> first_path_r = this->net_ptr->calculate_shortest_path(dummy_vertex_anchors, 
                                                                                                                  demand, 
                                                                                                                  *assignment_tree);
    
    if (first_path_r.empty()) {
        throw std::runtime_error(
            "demand can not reach any anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // demand --> target_anchor
    first_path_r.pop_front(); // ダミーノードを経路から削除する
    std::deque<std::pair<Net_2::vertex_descriptor, double>> first_path = first_path_r; // 経路を反転させる
    std::reverse(first_path.begin(), first_path.end());

    // 経路上にエンティティを認識できる地点があるか確認する
    Net_2::vertex_descriptor target_anchor = first_path.back().first; // 最寄りの拠点
    std::tuple<int, Net_2::vertex_descriptor, Net_2::vertex_descriptor> waystop_existance = find_first_entity(first_path);
    
    int first_entity_type;
    Net_2::vertex_descriptor way_stop;
    Net_2::vertex_descriptor assigned_entity;
    std::tie(first_entity_type, way_stop, assigned_entity) = waystop_existance;
    
    // 最初に認識するエンティティで場合分け
    size_t _; 
    double remaining_cost {0.0};
    switch (first_entity_type) {
        case -1:
            // std::cout << "\tNo waystop on the way to nearest anchor." << std::endl;        
            cost = first_path.front().second - first_path.back().second; // 需要点から拠点までの距離
            return std::make_pair(COST_PATTERN_Duncovered, cost);
            break;
        case 0:
            // std::cout << "\tFirst entity on the way to nearest anchor is anchor " << way_stop << "." << std::endl;
            
            // 需要点から中継地点までの距離を加算する
            cost += calculate_cost_on_path(demand, way_stop, first_path);
            
            // 中継地点からの距離を加算する
            std::tie(_, remaining_cost) = calculate_cost_to_visible_anchor(way_stop, assigned_entity);
            cost += remaining_cost;

            return std::make_pair(COST_PATTERN_DoutA, cost);
            
            break;
        case 1:
            // std::cout << "\tFirst entity on the way to nearest anchor is facility " << way_stop << "." << std::endl;
            
            // 需要点から中継地点までの距離を加算する
            cost += calculate_cost_on_path(demand, way_stop, first_path);

            // 中継地点からの距離を加算する
            std::tie(_, remaining_cost) = calculate_cost_to_visible_facility(way_stop, assigned_entity);
            cost += remaining_cost;

            return std::make_pair(COST_PATTERN_DoutF, cost);

            break;
        case 2:
            // std::cout << "\tFirst entity on the way to nearest anchor is sign " << way_stop << "." << std::endl;
            
            // 需要点から中継地点までの距離を加算する
            cost += calculate_cost_on_path(demand, way_stop, first_path);

            // 中継地点からの距離を加算する
            std::tie(_, remaining_cost) = calculate_cost_to_follow_signage(way_stop, assigned_entity);
            cost += remaining_cost;

            return std::make_pair(COST_PATTERN_DoutS, cost);
            
            break;
        default:
            throw std::runtime_error(
                "Invalid entity type found on the way to nearest anchor.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
    }
    
}

std::pair<bool, Net_2::vertex_descriptor> Net_SGFLP::find_waystop(Net_2::vertex_descriptor destination,
                                                                  const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const {
    //!注意
    //!pathは (start, total_distance) --> ... --> (destination, 0.0) の形式で、頂点とその頂点までの距離のペアの列であることを想定している
    // destinationがサービス供給点、サイン、拠点のどれであるかによって、
    // 可視範囲を確認する割当を切り替える
    // 優先順位は、拠点、サービス供給点、サイン
    const std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor>* destination_assignment_to_demand = nullptr;
    if (find_index(this->anchors, destination) != -1) {
        destination_assignment_to_demand = &this->anchor_assignment_to_demand;
    } else if (find_index(this->facilities, destination) != -1) {
        destination_assignment_to_demand = &this->facility_assignment_to_demand;
    } else if (find_index(this->signs, destination) != -1) {
        destination_assignment_to_demand = &this->sign_assignment_to_demand;
    } else {
        throw std::runtime_error(
            "Destination " + std::to_string(destination) + " has to be facility, sign or anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // destination を認識できる経路上の頂点を探索する
    for (const auto& v_on_the_way : path) {
        if (destination_assignment_to_demand->find(v_on_the_way.first) != destination_assignment_to_demand->end() &&
            (*destination_assignment_to_demand).at(v_on_the_way.first) == destination) {
            // destinationを認識できる途中の頂点が見つかった
            return std::make_pair(true, v_on_the_way.first);
        }
    }

    return std::make_pair(false, 0);

}

std::tuple<int, 
           Net_2::vertex_descriptor, 
           Net_2::vertex_descriptor> Net_SGFLP::find_first_entity(const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const {
    //!注意
    //!pathは (start, total_distance) --> ... --> (destination, 0.0) の形式で、頂点とその頂点までの距離のペアの列であることを想定している
    for (const auto& v_on_the_way : path) {
        if (this->anchor_assignment_to_demand.find(v_on_the_way.first) != this->anchor_assignment_to_demand.end()) {
            // 拠点
            return std::make_tuple(0, v_on_the_way.first, this->anchor_assignment_to_demand.at(v_on_the_way.first));
        } else if (this->facility_assignment_to_demand.find(v_on_the_way.first) != this->facility_assignment_to_demand.end()) {
            // サービス供給点
            return std::make_tuple(1, v_on_the_way.first, this->facility_assignment_to_demand.at(v_on_the_way.first));
        } else if (this->sign_assignment_to_demand.find(v_on_the_way.first) != this->sign_assignment_to_demand.end()) {
            // サイン
            return std::make_tuple(2, v_on_the_way.first, this->sign_assignment_to_demand.at(v_on_the_way.first));
        }
    }
    return std::make_tuple(-1, 0, 0);
}

double Net_SGFLP::calculate_cost_on_path(Net_2::vertex_descriptor s, 
                                         Net_2::vertex_descriptor t, 
                                         const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const {
    //!注意
    //!pathは (o, total_distance) --> ... --> (d, 0.0) の形式で、頂点とその頂点までの距離のペアの列であることを想定している
    double cost {0.0};
    double cost_s {-1.0};
    double cost_t {-1.0};
    for (const auto& v : path) {
        if (v.first == s) {
            cost_s = v.second;
        } 
        if (v.first == t) {
            cost_t = v.second;
        }
        if (cost_s != -1.0 && cost_t != -1.0) {
            // s, t両方の距離が見つかったらループを抜ける
            break;
        }
    }

    if (cost_s == -1.0 || cost_t == -1.0) {
        throw std::runtime_error(
            "Both start " + std::to_string(s) + " and target " + std::to_string(t) + " have to be on the path.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    cost = cost_s - cost_t; // s --> t の距離は、sまでの距離 - tまでの距離で求められる

    if (cost < 0.0) {
        throw std::runtime_error(
            "Cost on path from " + std::to_string(s) + " to " + std::to_string(t) + " cannot be negative.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    return cost;

}

//** Path Calculation Methods **//
std::pair<size_t, std::vector<Net_2::vertex_descriptor>> Net_SGFLP::calculate_path(Net_2::vertex_descriptor demand) const {
    std::pair<bool, Net_2::vertex_descriptor> assigned_facility = this->get_assigned_facility_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_sign = this->get_assigned_sign_to_demand(demand);
    std::pair<bool, Net_2::vertex_descriptor> assigned_anchor = this->get_assigned_anchor_to_demand(demand);

    if (assigned_anchor.first) {
        // 拠点が割り当てられている場合
        return calculate_path_to_visible_anchor(demand, assigned_anchor.second);
    } else if (assigned_facility.first) {
        // サービス供給点が割り当てられている場合
        return calculate_path_to_visible_facility(demand, assigned_facility.second);
    } else if (assigned_sign.first) {
        // サインが割り当てられている場合
        return calculate_path_to_follow_signage(demand, assigned_sign.second);
    } else {
        // 何も割り当てられていない場合
        return calculate_path_from_uncovered_demand(demand);
    }

}

std::pair<size_t, std::vector<Net_2::vertex_descriptor>> Net_SGFLP::calculate_path_to_visible_facility(Net_2::vertex_descriptor demand, 
                                                                                                       Net_2::vertex_descriptor facility) const {
    std::vector<Net_2::vertex_descriptor> path; // 需要点 --> サービス供給点への経路
    std::deque<std::pair<Net_2::vertex_descriptor, double>> path_r = this->net_ptr->calculate_shortest_path(facility, demand, this->facility_coverage_trees.at(facility));
    
    if (path_r.empty()) {   
        throw std::runtime_error(
            "No path from demand to assigned facility.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // 需要点 --> サービス供給点への経路を反転させる
    std::reverse(path_r.begin(), path_r.end());
    for (const auto& v : path_r) {
        path.push_back(v.first);
    }

    return std::make_pair(COST_PATTERN_DinF, path);
}

std::pair<size_t, std::vector<Net_2::vertex_descriptor>> Net_SGFLP::calculate_path_to_follow_signage(Net_2::vertex_descriptor demand, 
                                                                                                     Net_2::vertex_descriptor sign) const {
    std::vector<Net_2::vertex_descriptor> path; // 需要点 --> サイン --> サービス供給点 or 拠点への経路
    Net_2::vertex_descriptor curr_vertex = demand;
    Net_2::vertex_descriptor next_entity = this->navigation_assignment.at(sign);
    Net_2::vertex_descriptor last_entity;
    while (true) {
        // 次のエンティティへ向かう
        const std::vector<std::pair<std::size_t, double>>* shortest_path_tree = nullptr;
        if (find_index(this->anchors, next_entity) != -1) {
            shortest_path_tree = &this->anchor_shortest_path_trees.at(next_entity);
        } else if (find_index(this->signs, next_entity) != -1) {
            shortest_path_tree = &this->sign_shortest_path_trees.at(next_entity);
        } else if (find_index(this->facilities, next_entity) != -1) {
            shortest_path_tree = &this->facility_shortest_path_trees.at(next_entity);
        } else {
            throw std::runtime_error(
                "Next entity in navigation assignment has to be facility, sign or anchor.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
        }

        std::deque<std::pair<Net_2::vertex_descriptor, double>> path_r = this->net_ptr->calculate_shortest_path(next_entity, 
                                                                                                                curr_vertex, 
                                                                                                                *shortest_path_tree);
        
        if (path_r.empty()) {
            throw std::runtime_error(
                "No path from demand to sign.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );  
        }

        // curr_vertex --> next_entity
        std::reverse(path_r.begin(), path_r.end());

        std::pair<bool, Net_2::vertex_descriptor> waystop_existance = find_waystop(next_entity, path_r);
        std::vector<Net_2::vertex_descriptor> mid_path;
        if (waystop_existance.first) {
            // 中継地点がある場合は、そこまで向かう
            mid_path = calculate_path_between(curr_vertex, 
                                              waystop_existance.second, 
                                              path_r);
            path.insert(path.end(), mid_path.begin(), mid_path.end()); // 経路に中継地点までの経路を追加する
            curr_vertex = waystop_existance.second;
        } else {
            // 次のエンティティまで向かう
            mid_path = calculate_path_between(curr_vertex, 
                                              next_entity, 
                                              path_r);
            path.insert(path.end(), mid_path.begin(), mid_path.end()); // 経路に次のエンティティまでの経路を追加する
            curr_vertex = next_entity;
        }

        // 次のエンティティを更新
        if (this->navigation_assignment.find(next_entity) == this->navigation_assignment.end()) {
            // next_entityが見つからない場合はループを抜ける
            
            // next_entityがない場合はサービス供給点、拠点のいずれかであるはずなので、どちらにも見つからない場合はエラー
            if (find_index(this->facilities, next_entity) == -1 && 
                find_index(this->anchors, next_entity) == -1) {
                throw std::runtime_error(
                    "Last entity in navigation assignment is not found in facilities or anchors.\n"
                    "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
                );
            }

            last_entity = next_entity;
            break;
        }

        next_entity = this->navigation_assignment.at(next_entity);
        
    }

    // 現在地から最後のエンティティまでの経路を追加する
    const std::vector<std::pair<std::size_t, double>>* shortest_path_tree = nullptr;
    if (find_index(this->anchors, last_entity) != -1) {
        shortest_path_tree = &this->anchor_shortest_path_trees.at(last_entity);
    } else if (find_index(this->signs, last_entity) != -1) {
        shortest_path_tree = &this->sign_shortest_path_trees.at(last_entity);
    } else if (find_index(this->facilities, last_entity) != -1) {
        shortest_path_tree = &this->facility_shortest_path_trees.at(last_entity);
    } else {
        throw std::runtime_error(
            "Next entity in navigation assignment has to be facility, sign or anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    std::deque<std::pair<Net_2::vertex_descriptor, double>> final_path_r = this->net_ptr->calculate_shortest_path(last_entity, 
                                                                                                                  curr_vertex, 
                                                                                                                  *shortest_path_tree);
    // curr_vertex --> last_entity
    std::deque<std::pair<Net_2::vertex_descriptor, double>> final_path = final_path_r; // 経路を反転させる
    std::reverse(final_path.begin(), final_path.end());
    std::vector<Net_2::vertex_descriptor> final_path_vec = calculate_path_between(curr_vertex, last_entity, final_path);
    path.insert(path.end(), final_path_vec.begin(), final_path_vec.end()); // 経路に現在地から最後のエンティティまでの経路を追加する

    return std::make_pair(COST_PATTERN_DinS, path);
}

std::pair<size_t, std::vector<Net_2::vertex_descriptor>> Net_SGFLP::calculate_path_to_visible_anchor(Net_2::vertex_descriptor demand, 
                                                                                                     Net_2::vertex_descriptor anchor) const {
    std::vector<Net_2::vertex_descriptor> path; // 需要点 --> 拠点への経路
    std::deque<std::pair<Net_2::vertex_descriptor, double>> path_r = this->net_ptr->calculate_shortest_path(anchor, demand, this->anchor_coverage_trees.at(anchor));
    
    if (path_r.empty()) {   
        throw std::runtime_error(
            "No path from demand to assigned anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // 需要点 --> 拠点への経路を反転させる
    std::reverse(path_r.begin(), path_r.end());
    for (const auto& v : path_r) {
        path.push_back(v.first);
    }

    return std::make_pair(COST_PATTERN_DinA, path);
    
}

std::pair<size_t, std::vector<Net_2::vertex_descriptor>> Net_SGFLP::calculate_path_from_uncovered_demand(Net_2::vertex_descriptor demand) const {
    std::vector<Net_2::vertex_descriptor> path;
    
    // まず最初に、最寄りの拠点に向かう
    const std::vector<std::pair<Net_2::vertex_descriptor, double>>* assignment_tree = &this->anchor_shortest_path_tree;
    std::deque<std::pair<Net_2::vertex_descriptor, double>> first_path_r = this->net_ptr->calculate_shortest_path(dummy_vertex_anchors, 
                                                                                                                  demand, 
                                                                                                                  *assignment_tree);
    
    if (first_path_r.empty()) {
        throw std::runtime_error(
            "demand can not reach any anchor.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

    // demand --> target_anchor
    first_path_r.pop_front(); // ダミーノードを経路から削除する
    std::deque<std::pair<Net_2::vertex_descriptor, double>> first_path = first_path_r; // 経路を反転させる
    std::reverse(first_path.begin(), first_path.end());

    // 経路上にエンティティを認識できる地点があるか確認する
    Net_2::vertex_descriptor target_anchor = first_path.back().first; // 最寄りの拠点
    std::tuple<int, Net_2::vertex_descriptor, Net_2::vertex_descriptor> waystop_existance = find_first_entity(first_path);
    
    int first_entity_type;
    Net_2::vertex_descriptor way_stop;
    Net_2::vertex_descriptor assigned_entity;
    std::tie(first_entity_type, way_stop, assigned_entity) = waystop_existance;
    
    // 最初に認識するエンティティで場合分け
    size_t _; 
    std::vector<Net_2::vertex_descriptor> remaining_path;
    switch (first_entity_type) {
        case -1:
            path = calculate_path_between(demand, target_anchor, first_path);
            return std::make_pair(COST_PATTERN_Duncovered, path);
            break;
        case 0:
            // 需要点から中継地点までの距離を加算する
            path = calculate_path_between(demand, way_stop, first_path);
            
            // 中継地点からの距離を加算する
            std::tie(_, remaining_path) = calculate_path_to_visible_anchor(way_stop, assigned_entity);
            path.insert(path.end(), remaining_path.begin(), remaining_path.end());

            return std::make_pair(COST_PATTERN_DoutA, path);
            
            break;
        case 1:
            // 需要点から中継地点までの距離を加算する
            path = calculate_path_between(demand, way_stop, first_path);

            // 中継地点からの距離を加算する
            std::tie(_, remaining_path) = calculate_path_to_visible_facility(way_stop, assigned_entity);
            path.insert(path.end(), remaining_path.begin(), remaining_path.end());

            return std::make_pair(COST_PATTERN_DoutF, path);

            break;
        case 2:
            // 需要点から中継地点までの距離を加算する
            path = calculate_path_between(demand, way_stop, first_path);

            // 中継地点からの距離を加算する
            std::tie(_, remaining_path) = calculate_path_to_follow_signage(way_stop, assigned_entity);
            path.insert(path.end(), remaining_path.begin(), remaining_path.end());

            return std::make_pair(COST_PATTERN_DoutS, path);
            
            break;
        default:
            throw std::runtime_error(
                "Invalid entity type found on the way to nearest anchor.\n"
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );
    }

}

std::vector<Net_2::vertex_descriptor> Net_SGFLP::calculate_path_between(Net_2::vertex_descriptor s,
                                                                        Net_2::vertex_descriptor t,
                                                                        const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const {
    std::vector<Net_2::vertex_descriptor> result;
    bool start_adding = false;
    for (const auto& p : path) {
        if (p.first == s) {
            start_adding = true;
        }
        if (start_adding) {
            result.push_back(p.first);
        }
        if (p.first == t) {
            break;
        }
    }
    return result;
}