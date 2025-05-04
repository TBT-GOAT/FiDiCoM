// include header
#include "domain/flp/net_wp.h"

// include stl util
#include "core/util/std_vector_util.h"
#include "core/util/random_engine.h"

//** Static Member Variable **//
static std::uniform_real_distribution<double> dist(0.0, 1.0);
const Point_2 Net_WP::DUMMY_POINT = Point_2(dist(Random_Engine::get_engine()), dist(Random_Engine::get_engine()));

//** Constructor **//
Net_WP::Net_WP() : net_ptr(std::make_shared<Net_2>()) {} 
Net_WP::Net_WP(std::shared_ptr<Net_2> net_ptr) : net_ptr(net_ptr) {
    if (!this->net_ptr) {
        throw std::runtime_error("net_ptr is not initialized.\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

//** Getter **//
Point_2 Net_WP::get_inner_point() const {
    return inner_point;
}
std::vector<Net_2::vertex_descriptor> Net_WP::get_demands() const {
    return demands;
}
std::vector<Net_2::vertex_descriptor> Net_WP::get_facilities() const {
    return  facilities;
}
Net_2::vertex_descriptor Net_WP::get_dummy_vertex_facilities() const {
    return dummy_vertex_facilities;
}
std::vector<std::pair<Net_2::vertex_descriptor, double>> Net_WP::get_facility_shortest_path_tree() const {
    return facility_shortest_path_tree;
}

//** Setter **//
void Net_WP::set_inner_point(const Point_2 inner_point) {
    this->inner_point = inner_point;
}
void Net_WP::set_demands() {
    std::unordered_set<Net_2::vertex_descriptor> demands_set;
    demands_set = this->net_ptr->calculate_reachable_vertices(this->inner_point);
    this->demands = std::vector<Net_2::vertex_descriptor>(demands_set.begin(), demands_set.end());
}
void Net_WP::set_demands(const Point_2 inner_point) {
    set_inner_point(inner_point);
    set_demands();
}
void Net_WP::set_demands(const std::vector<Net_2::vertex_descriptor> demands) {
    this->demands = demands;
}
void Net_WP::set_facilities(const std::vector<Net_2::vertex_descriptor> facilities) {
    this->facilities = facilities;
}
void Net_WP::set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities) {
    this->dummy_vertex_facilities = dummy_vertex_facilities;
}

//** Network Methods **//

void Net_WP::build_tree() {
    
    build_facility_shortest_path_tree();
}

void Net_WP::clear_tree() {

    facility_shortest_path_tree.clear();
    
}

void Net_WP::insert_dummy_vertex_facilities() {
    
    std::shared_ptr<Node_2> dummy_node_facilities_ptr = std::make_shared<Node_2>(DUMMY_POINT);

    Net_2::vertex_descriptor dummy_vertex_facilities = boost::add_vertex(dummy_node_facilities_ptr, *(this->net_ptr));
    this->set_dummy_vertex_facilities(dummy_vertex_facilities);

    for (const auto& facility_vertex : this->facilities) {
        std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], 
                                                                          (*(this->net_ptr))[facility_vertex]);
        dummy_edge_ptr->set_is_dummy(true);
        boost::add_edge(this->dummy_vertex_facilities, facility_vertex, dummy_edge_ptr, *(this->net_ptr));
    }

}

void Net_WP::remove_dummy_vertex() {
    if (this->dummy_vertex_facilities != UNINITIALIZED_DUMMY_VERTEX) {
        this->net_ptr->remove_vertex(this->dummy_vertex_facilities);
    }

    // 無効な値にリセット
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX;
}

void Net_WP::remove_dummy_vertex_facilities() {
    boost::clear_vertex(dummy_vertex_facilities, *(this->net_ptr));
    boost::remove_vertex(dummy_vertex_facilities, *(this->net_ptr));
    
    // ダミー頂点を無効な値に設定
    dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX;
}

void Net_WP::update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                            const Net_2::vertex_descriptor next_facility_vertex) 
{
    boost::remove_edge(dummy_vertex_facilities, prev_facility_vertex, *(this->net_ptr));

    std::shared_ptr<Edge_2> dummy_edge_ptr = std::make_shared<Edge_2>((*(this->net_ptr))[this->dummy_vertex_facilities], 
                                                                      (*(this->net_ptr))[next_facility_vertex]);
    dummy_edge_ptr->set_is_dummy(true);
    boost::add_edge(this->dummy_vertex_facilities, next_facility_vertex, dummy_edge_ptr, *(this->net_ptr));
}

void Net_WP::build_facility_shortest_path_tree() {

    this->facility_shortest_path_tree = this->net_ptr->calculate_shortest_path_tree(this->get_dummy_vertex_facilities(), 
                                                                                    Net_2::MODE_ROUTE, 
                                                                                    true, 
                                                                                    true);

}

//** Initialization Methods **//
std::vector<Net_2::vertex_descriptor> Net_WP::search_nearest_demands(const std::vector<Point_2> points) {
    std::vector<Net_2::vertex_descriptor> nearest_demands {};
    for (const auto& p : points) {
        Net_2::vertex_descriptor nearest_vertex = net_ptr->find_nearest_node(p);
        if (find_index(demands, nearest_vertex) != -1) {
            // pに最近隣の頂点が需要点の中にある
            nearest_demands.push_back(nearest_vertex);
        } else {
            // pに最近隣の頂点が需要点の中にない
            // 最近隣の頂点から順に需要点を探索する
            std::vector<Net_2::vertex_descriptor> prohibited_vertices {dummy_vertex_facilities};
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
                    nearest_demands.push_back(v_and_d.first);
                }
            }

            std::cerr << "Could not find nearest demand to " << p << std::endl;
            throw std::runtime_error(
                "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
            );

        }
    }

    return nearest_demands;
    
}

std::vector<Net_2::vertex_descriptor> Net_WP::select_demands_at_random(const size_t selected_num) {
    // 重複無しの無作為抽出
    return random_sampling(demands, selected_num, false);
}

void Net_WP::initialize_facilities(const std::vector<Point_2> facility_points) {
    this->facilities = search_nearest_demands(facility_points);
    
    if (this->dummy_vertex_facilities == UNINITIALIZED_DUMMY_VERTEX) {
        this->insert_dummy_vertex_facilities();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing facilities.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }
    
}

void Net_WP::initialize_facilities(const size_t facility_num) {
    if (facility_num == 0) {
        this->facilities = std::vector<Net_2::vertex_descriptor>();
    } else {
        this->facilities = select_demands_at_random(facility_num);
    }

    if (this->dummy_vertex_facilities == UNINITIALIZED_DUMMY_VERTEX) {
        this->insert_dummy_vertex_facilities();
    } else {
        throw std::runtime_error(
            "Clear dummy vertex before initializing facilities.\n"
            "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__)
        );
    }

}

//** Finalization Methods **//
void Net_WP::clear() {
    remove_dummy_vertex();
}

//** Cost Function Methods **//
double Net_WP::calculate_cost(Net_2::vertex_descriptor demand) const {
    return this->facility_shortest_path_tree.at(demand).second;
}