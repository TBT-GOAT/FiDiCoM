


// include header
#include "core/geom/spherical_geom_util.h"


double calc_angle(const Vector_2& v1, const Vector_2& v2) {
    double dot_product = v1 * v2;
    double length_v1 = std::sqrt(v1.squared_length());
    double length_v2 = std::sqrt(v2.squared_length());

    double angle = std::acos(std::clamp(dot_product / (length_v1 * length_v2), -1.0, 1.0));

    return angle;
}

double calc_angle(const Point_2& center_point, const Point_2& target_point) {
    double dx = target_point.x() - center_point.x();
    double dy = target_point.y() - center_point.y();

    double angle = std::atan2(dy, dx);

    // [0, 2pi) に変換
    if (angle < 0) {
        angle += 2 * std::acos(-1);
    }

    return angle;
}

double calc_angle(const Vector_3& v1, const Vector_3& v2) {
    double dot_product = v1 * v2;
    double length_v1 = std::sqrt(v1.squared_length());
    double length_v2 = std::sqrt(v2.squared_length());

    double angle = std::acos(std::clamp(dot_product / (length_v1 * length_v2), -1.0, 1.0));

    return angle;
}

double calc_inner_angle_on_sphere(const Point_3& center_point, 
                                  const Point_on_sphere_3& curr_point, 
                                  const Point_on_sphere_3& prev_point, 
                                  const Point_on_sphere_3& next_point)
{
    Vector_3 curr_vec = curr_point - center_point;
    Vector_3 prev_vec = prev_point - center_point;
    Vector_3 next_vec = next_point - center_point;

    Vector_3 prev_dir_vec_rotate90 = CGAL::cross_product(prev_vec, curr_vec); // curr_pointとprev_pointが乗る大円の法線ベクトル
    Vector_3 next_dir_vec_rotate90 = CGAL::cross_product(next_vec, curr_vec); // curr_pointとnext_pointが乗る大円の法線ベクトル

    double inner_angle = calc_angle(prev_dir_vec_rotate90, next_dir_vec_rotate90);

    return inner_angle;

}

double calc_center_angle (const Point_2& center_point, 
                          const Point_2& from_point, 
                          const Point_2& to_point)
{
    Vector_2 v1 = from_point - center_point;
    Vector_2 v2 = to_point - center_point;
    return calc_angle(v1, v2);
}

double calc_solid_angle(const Point_3& center_point, 
                        const std::vector<Point_on_sphere_3>& vertices, 
                        double radius) 
{
    size_t n = vertices.size();
    
    if (n < 3) {
        throw std::invalid_argument("A spherical polygon must have at least 3 vertices.\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    double solid_angle_sum {0.0};
    for (size_t i{0}; i < n; ++i) {
        const Point_on_sphere_3 prev_point = vertices[(i + n - 1) % n];
        const Point_on_sphere_3 curr_point = vertices[i];
        const Point_on_sphere_3 next_point = vertices[(i + 1) % n];

        double inner_angle = calc_inner_angle_on_sphere(center_point, curr_point, prev_point, next_point);
        solid_angle_sum += inner_angle;
    }

    return solid_angle_sum - (n - 2) * std::acos(-1); // ガウス-ボンネの定理

}

std::unordered_map<Point_2, std::pair<Point_2, Point_2>> construct_circular_Voronoi_diagram(const Point_2& center_point, 
                                                                                            const std::vector<Point_2>& generators, 
                                                                                            const double radius)
{
    // 母点を中心角でソート
    std::vector<Point_2> sorted_generators = generators;
    std::sort(
        sorted_generators.begin(), 
        sorted_generators.end(), 
        [&center_point](const Point_2& a, const Point_2& b) {
            return calc_angle(center_point, a) < calc_angle(center_point, b);   
        }
    );

    // 隣り合う母点の角の二等分がボロノイ頂点
    size_t generator_num = sorted_generators.size();
    std::vector<Point_2> Voronoi_vertices(generator_num);
    for (size_t i {0}; i < generator_num - 1; ++i) {
        const Point_2& curr_generator = sorted_generators[i];
        const Point_2& next_generator = sorted_generators[(i + 1) % generator_num];
        double angle = (calc_angle(center_point, curr_generator) + calc_angle(center_point, next_generator)) / 2;
        Voronoi_vertices[i] = Point_2(radius * std::cos(angle), radius * std::sin(angle));
    }

    // 最後と最初の母点の角の二等分
    const Point_2& curr_generator = sorted_generators[generator_num - 1];
    const Point_2& next_generator = sorted_generators[0];
    double angle = (calc_angle(center_point, curr_generator) + (calc_angle(center_point, next_generator) + 2 * std::acos(-1))) / 2;
    Voronoi_vertices[generator_num - 1] = Point_2(std::cos(angle), std::sin(angle));

    // ボロノイ頂点を結んでセル（円弧の両端点のペア）を作る
    std::unordered_map<Point_2, std::pair<Point_2, Point_2>> circular_Voronoi_diagram;
    for (size_t i {0}; i < generator_num; ++i) {
        Point_2 from_vertex = Voronoi_vertices[(i + generator_num - 1) % generator_num];
        Point_2 to_vertex = Voronoi_vertices[i];
        circular_Voronoi_diagram[sorted_generators[i]] = std::pair(from_vertex, to_vertex);
    }

    return circular_Voronoi_diagram;

}

std::unordered_map<Point_3, std::vector<Point_on_sphere_3>> construct_spherical_Voronoi_diagram(const Point_3& center_point, 
                                                                                                const std::vector<Point_3>& generators, 
                                                                                                const double radius) 
{
    Traits sphere(center_point, radius);
    DToS2 dtos(sphere);

    Traits::Construct_point_on_sphere_2 cpos = sphere.construct_point_on_sphere_2_object();

    // 母点の追加
    for (const auto& gen : generators) {
        dtos.insert(cpos(gen)); // 球面に投影する
    }

    std::unordered_map<Point_3, std::vector<Point_on_sphere_3>> spherical_Voronoi_diagram;

    for (auto vit = dtos.finite_vertices_begin(); vit != dtos.finite_vertices_end(); ++vit) {
        
        std::vector<Point_on_sphere_3> points_on_sphere;
        DToS2::Face_circulator fc = dtos.incident_faces(vit), done(fc);
        
        if (dtos.is_infinite(fc)) {
            throw std::runtime_error("Invalid Voronoi cell.\n"
                                     "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        if (fc != nullptr) {
            do {
                Point_on_sphere_3 curr_point = dtos.dual_on_sphere(fc);
                if (points_on_sphere.empty() || curr_point != points_on_sphere.back()) {
                    points_on_sphere.push_back(dtos.dual_on_sphere(fc)); // 球面上のボロノイ頂点
                } // else 縮退している
            } while (++fc != done);
        }

        spherical_Voronoi_diagram[vit->point()] = points_on_sphere;

    }

    return spherical_Voronoi_diagram;

}