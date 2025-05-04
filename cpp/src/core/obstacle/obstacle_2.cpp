
// include header
#include "core/obstacle/obstacle_2.h"

// include Edge_2
#include "core/edge/edge_2.h"

// include standard
#include <filesystem>

typedef K::Point_2 Point_2_; // Segment_2のprivateメンバ変数と区別するため

//** Constructor **//
Obstacle_2::Obstacle_2(const Segment_2 segment_2) : 
    Segment_2(segment_2)
{}

Obstacle_2::Obstacle_2(const Segment_2 segment_2, 
                       const bool is_transparent, 
                       const bool is_navigable, 
                       const bool is_domain, 
                       const std::string name) :
    Segment_2(segment_2), 
    is_transparent(is_transparent),
    is_navigable(is_navigable), 
    is_domain(is_domain), 
    name(name)
{}

Obstacle_2::Obstacle_2(const Segment_2 segment_2, 
                       const bool is_transparent, 
                       const bool is_navigable, 
                       std::unordered_set<std::shared_ptr<Edge_2>>&& x_edge_ptrs, 
                       const bool is_domain, 
                       const std::string name) :
    Segment_2(segment_2), 
    is_transparent(is_transparent),
    is_navigable(is_navigable),
    x_edge_ptrs(std::move(x_edge_ptrs)), 
    is_domain(is_domain), 
    name(name)
{}

//** Getter **//
bool Obstacle_2::get_is_transparent() const {
    return is_transparent;
}

bool Obstacle_2::get_is_navigable() const {
    return is_navigable;
}

std::unordered_set<std::shared_ptr<Edge_2>> Obstacle_2::get_x_edge_ptrs() const {
    return x_edge_ptrs;
}

std::string Obstacle_2::get_name() const {
    return name;
}

//** Setter **//
void Obstacle_2::set_is_transparent(bool is_transparent) {
    this->is_transparent = is_transparent;
}

void Obstacle_2::set_is_navigable(bool is_navigable) {
    this->is_navigable = is_navigable;
}

void Obstacle_2::set_x_edge_ptrs(std::unordered_set<std::shared_ptr<Edge_2>>&& x_edge_ptrs) {
    this->x_edge_ptrs = std::move(x_edge_ptrs);
}

void Obstacle_2::set_name(std::string name) {
    this->name = name;
}

//** Construction Method **//
std::vector<std::shared_ptr<Obstacle_2>> Obstacle_2::convert_polygon(const Polygon_2& polygon,  
                                                                     const bool is_transparent, 
                                                                     const bool is_navigable, 
                                                                     const bool is_domain, 
                                                                     const std::string name) {
    
    std::vector<std::shared_ptr<Obstacle_2>> obstacle_ptrs(polygon.size());
    
    std::transform(
        polygon.edges_begin(), 
        polygon.edges_end(), 
        obstacle_ptrs.begin(),
        [is_transparent, is_navigable, is_domain, name](const Segment_2& e) -> std::shared_ptr<Obstacle_2> {
            return std::make_shared<Obstacle_2>(e, is_transparent, is_navigable, is_domain, name);
        }
    );

    return obstacle_ptrs;
}

void Obstacle_2::insert_x_edge_ptr(std::shared_ptr<Edge_2> edge_ptr) {
    this->x_edge_ptrs.insert(edge_ptr);
}

void Obstacle_2::erase_x_edge_ptr(std::shared_ptr<Edge_2> edge_ptr) {
    this->x_edge_ptrs.erase(edge_ptr);
}

//** Geometric Method **//
bool Obstacle_2::is_intersecting(const Segment_2 segment_2) const {
    return CGAL::do_intersect(*this, segment_2);
}

//** File IO Method **//
std::vector<std::shared_ptr<Obstacle_2>> Obstacle_2::read_obstacles(const std::string& abs_file_path, 
                                                                    const bool is_transparent, 
                                                                    const bool is_navigable, 
                                                                    const bool is_domain) {
    std::ifstream file(abs_file_path);
    std::string obstacle_name = std::filesystem::path(abs_file_path).stem().string(); // ファイル名 = 障害物の名前
    std::vector<std::shared_ptr<Obstacle_2>> obstacle_ptrs;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return obstacle_ptrs;
    }

    std::string line;
    double x1;
    double y1;
    double x2;
    double y2;
    while (std::getline(file, line)) {
        std::istringstream line_stream(line);
        line_stream >> x1 >> y1 >> x2 >> y2;

        std::shared_ptr<Obstacle_2> obstacle_ptr = std::make_shared<Obstacle_2>(Segment_2(Point_2_(x1, y1), Point_2_(x2, y2)), 
                                                                                is_transparent, 
                                                                                is_navigable, 
                                                                                is_domain, 
                                                                                obstacle_name);
        obstacle_ptrs.push_back(obstacle_ptr);

    }
    
    return obstacle_ptrs;

}

std::vector<std::shared_ptr<Obstacle_2>> Obstacle_2::read_obstacles(const std::string& abs_file_path, 
                                                                    const bool is_domain) {
    std::ifstream file(abs_file_path);
    std::string obstacle_name = std::filesystem::path(abs_file_path).stem().string(); // ファイル名 = 障害物の名前
    std::vector<std::shared_ptr<Obstacle_2>> obstacle_ptrs;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return obstacle_ptrs;
    }

    std::string line;
    double x1;
    double y1;
    double x2;
    double y2;
    bool transparency;
    bool navigability;
    while (std::getline(file, line)) {
        std::istringstream line_stream(line);
        line_stream >> x1 >> y1 >> x2 >> y2 >> transparency >> navigability;

        std::shared_ptr<Obstacle_2> obstacle_ptr = std::make_shared<Obstacle_2>(Segment_2(Point_2_(x1, y1), Point_2_(x2, y2)), 
                                                                                transparency, 
                                                                                navigability, 
                                                                                is_domain, 
                                                                                obstacle_name);
        obstacle_ptrs.push_back(obstacle_ptr);

    }
    
    return obstacle_ptrs;

}