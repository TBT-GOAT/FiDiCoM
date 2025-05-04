

// include standards
#include <unordered_map>

// include header
#include "core/net/rDn_2.h"

//** Network Method **//
void rDn_2::initialize() {
    // ドロネー分割を実行する
    std::vector<std::shared_ptr<Node_2>> node_ptrs = generate_random_nodes();

    for (auto node_ptr : node_ptrs) {
        tessellation.insert(*node_ptr);
    }

    // ノードを生成し，Graph_2::vertex_descriptorに割り当てる
    std::unordered_map<Point_2, Graph_2::vertex_descriptor> map_pt_2_vertex;
    for (Delaunay_2::Point_iterator pit = tessellation.points_begin(); pit != tessellation.points_end(); ++pit) {
        std::shared_ptr<Node_2> node_ptr = std::make_shared<Node_2>(*pit);
        Graph_2::vertex_descriptor vertex = boost::add_vertex(node_ptr, *this);
        map_pt_2_vertex[*pit] = vertex;
    }  
        
    // ドロネー分割をネットワークに変換する
    for (Delaunay_2::Edge_iterator eit = tessellation.edges_begin(); eit != tessellation.edges_end(); ++eit) {
        // eit は std::pair<face, vertex index>
        // eit->second は eitの反対にある頂点を指す（0, 1, 2のいずれかの番号）
        // したがって，＋1，+2 した頂点が eit の端点
        Delaunay_2::Vertex_handle v1 = eit->first->vertex((eit->second + 1) % 3);
        Delaunay_2::Vertex_handle v2 = eit->first->vertex((eit->second + 2) % 3);
        Point_2 p1 = v1->point();
        Point_2 p2 = v2->point();

        Graph_2::vertex_descriptor vertex1 = map_pt_2_vertex[p1];
        Graph_2::vertex_descriptor vertex2 = map_pt_2_vertex[p2];

        std::shared_ptr<Edge_2> edge_ptr = std::make_shared<Edge_2>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }

}

void rDn_2::initialize(const std::vector<std::shared_ptr<Node_2>> node_ptrs) {
    
    node_num = node_ptrs.size();
    
    for (auto node_ptr : node_ptrs) {
        tessellation.insert(*node_ptr);
    }

    // ノードを生成し，Graph_2::vertex_descriptorに割り当てる
    std::unordered_map<Point_2, Graph_2::vertex_descriptor> map_pt_2_vertex;
    for (Delaunay_2::Point_iterator pit = tessellation.points_begin(); pit != tessellation.points_end(); ++pit) {
        std::shared_ptr<Node_2> node_ptr = std::make_shared<Node_2>(*pit);
        Graph_2::vertex_descriptor vertex = boost::add_vertex(node_ptr, *this);
        map_pt_2_vertex[*pit] = vertex;
    }  
        
    // ドロネー分割をネットワークに変換する
    for (Delaunay_2::Edge_iterator eit = tessellation.edges_begin(); eit != tessellation.edges_end(); ++eit) {
        // eit は std::pair<face, vertex index>
        // eit->second は eitの反対にある頂点を指す（0, 1, 2のいずれかの番号）
        // したがって，＋1，+2 した頂点が eit の端点
        Delaunay_2::Vertex_handle v1 = eit->first->vertex((eit->second + 1) % 3);
        Delaunay_2::Vertex_handle v2 = eit->first->vertex((eit->second + 2) % 3);
        Point_2 p1 = v1->point();
        Point_2 p2 = v2->point();

        Graph_2::vertex_descriptor vertex1 = map_pt_2_vertex[p1];
        Graph_2::vertex_descriptor vertex2 = map_pt_2_vertex[p2];

        std::shared_ptr<Edge_2> edge_ptr = std::make_shared<Edge_2>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }

}

//** Geometric Method **//
Delaunay_2::Vertex_handle rDn_2::search_geom_vertex(const std::shared_ptr<Node_2> node_ptr) const {
    Delaunay_2::Vertex_handle vh;
    for (auto vit = tessellation.finite_vertices_begin(); vit != tessellation.finite_vertices_end(); ++vit) {
        if (*node_ptr == vit->point()) {
            vh = vit;
            break;
        }
    }

    return vh;
}

rDn_2::vertex_descriptor rDn_2::search_net_vertex(const Delaunay_2::Vertex_handle vh) const {
    rDn_2::vertex_descriptor vertex_desc;

    rDn_2::vertex_iterator vit, vit_end;
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        if (vh->point() == *(*this)[*vit]) {
            vertex_desc = *vit;
            break;
        }
    }

    return vertex_desc;
}

Net_2::vertex_descriptor rDn_2::find_nearest_node(Point_2 p) const {
    Delaunay_2::Vertex_handle nearest_vertex = tessellation.nearest_vertex(p);
    return search_net_vertex(nearest_vertex);
}

std::vector<Point_2> rDn_2::build_cell(const std::shared_ptr<Node_2> node_ptr) const {
    Delaunay_2::Vertex_handle vh = search_geom_vertex(node_ptr);

    std::vector<Point_2> voronoi_cell;
    std::vector<Point_2> open_voronoi_cell;
    Delaunay_2::Face_circulator fc = tessellation.incident_faces(vh), done(fc);

    if (fc != nullptr) {
        do {

            if (tessellation.is_infinite(fc)) {
                // セルが閉じない
                return open_voronoi_cell;
            }

            voronoi_cell.push_back(tessellation.dual(fc));
        } while (++fc != done);
    }

    return voronoi_cell;

}

//** File IO Method **//
void rDn_2::write_cells(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_2>> node_ptrs = collect_node_ptrs();

    for (auto& node_ptr : node_ptrs) {
        std::vector<Point_2> voronoi_cell = build_cell(node_ptr);
        for (auto p : voronoi_cell) {
            f << std::scientific 
            << std::setprecision(std::numeric_limits<double>::max_digits10) 
            << " " << p.x() << " " << p.y();
        }

        f << std::endl;

    }
    
}