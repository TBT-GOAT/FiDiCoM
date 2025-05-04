

// include standards
#include <unordered_map>

// include CGAL
#include <CGAL/convex_hull_3.h>

// include header
#include "core/net/rDn_3.h"

// include standard util
#include "core/util/std_string_util.h"


//** Network Method **//
void rDn_3::initialize() {
    // ドロネー分割を実行する
    std::vector<std::shared_ptr<Node_3>> node_ptrs = generate_random_nodes();

    for (auto node_ptr : node_ptrs) {
        tessellation.insert(*node_ptr);
    }

    // ノードを生成し，Graph_3::vertex_descriptorに割り当てる
    std::unordered_map<Point_3, Graph_3::vertex_descriptor> map_pt_3_vertex;
    for (Delaunay_3::Point_iterator pit = tessellation.points_begin(); pit != tessellation.points_end(); ++pit) {
        std::shared_ptr<Node_3> node_ptr = std::make_shared<Node_3>(*pit);
        Graph_3::vertex_descriptor vertex = boost::add_vertex(node_ptr, *this);
        map_pt_3_vertex[*pit] = vertex;
    }  
        
    // ドロネー分割をネットワークに変換する
    for (Delaunay_3::Finite_edges_iterator eit = tessellation.finite_edges_begin(); eit != tessellation.finite_edges_end(); ++eit) {
        // https://doc.cgal.org/5.4/TDS_3/classTriangulationDataStructure__3.html#af31db7673a6d7d28c0bb90a3115ac695
        // eit は std::pair<facet, vertex index>
        // eit->second は eitの片方の頂点を指す（0, 1, 2, 3のいずれかの番号）
        // eit->third は eitのもう片方の頂点を指す（0, 1, 2, 3のいずれかの番号）
        Delaunay_3::Vertex_handle v1 = eit->first->vertex(eit->second);
        Delaunay_3::Vertex_handle v2 = eit->first->vertex(eit->third);
        Point_3 p1 = v1->point();
        Point_3 p2 = v2->point();

        Graph_3::vertex_descriptor vertex1 = map_pt_3_vertex[p1];
        Graph_3::vertex_descriptor vertex2 = map_pt_3_vertex[p2];

        std::shared_ptr<Edge_3> edge_ptr = std::make_shared<Edge_3>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }

}

void rDn_3::initialize(const std::vector<std::shared_ptr<Node_3>> node_ptrs) {
    
    node_num = node_ptrs.size();

    for (auto node_ptr : node_ptrs) {
        tessellation.insert(*node_ptr);
    }

    // ノードを生成し，Graph_3::vertex_descriptorに割り当てる
    std::unordered_map<Point_3, Graph_3::vertex_descriptor> map_pt_3_vertex;
    for (Delaunay_3::Point_iterator pit = tessellation.points_begin(); pit != tessellation.points_end(); ++pit) {
        std::shared_ptr<Node_3> node_ptr = std::make_shared<Node_3>(*pit);
        Graph_3::vertex_descriptor vertex = boost::add_vertex(node_ptr, *this);
        map_pt_3_vertex[*pit] = vertex;
    }  
        
    // ドロネー分割をネットワークに変換する
    for (Delaunay_3::Finite_edges_iterator eit = tessellation.finite_edges_begin(); eit != tessellation.finite_edges_end(); ++eit) {
        // https://doc.cgal.org/5.4/TDS_3/classTriangulationDataStructure__3.html#af31db7673a6d7d28c0bb90a3115ac695
        // eit は std::pair<facet, vertex index>
        // eit->second は eitの片方の頂点を指す（0, 1, 2, 3のいずれかの番号）
        // eit->third は eitのもう片方の頂点を指す（0, 1, 2, 3のいずれかの番号）
        Delaunay_3::Vertex_handle v1 = eit->first->vertex(eit->second);
        Delaunay_3::Vertex_handle v2 = eit->first->vertex(eit->third);
        Point_3 p1 = v1->point();
        Point_3 p2 = v2->point();

        Graph_3::vertex_descriptor vertex1 = map_pt_3_vertex[p1];
        Graph_3::vertex_descriptor vertex2 = map_pt_3_vertex[p2];

        std::shared_ptr<Edge_3> edge_ptr = std::make_shared<Edge_3>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }
}

//** Geometric Method **//
Delaunay_3::Vertex_handle rDn_3::search_geom_vertex(const std::shared_ptr<Node_3> node_ptr) const {
    Delaunay_3::Vertex_handle vh;
    for (auto vit = tessellation.finite_vertices_begin(); vit != tessellation.finite_vertices_end(); ++vit) {
        if (*node_ptr == vit->point()) {
            vh = vit;
            break;
        }
    }

    return vh;
}

rDn_3::vertex_descriptor rDn_3::search_net_vertex(const Delaunay_3::Vertex_handle vh) const {
    rDn_3::vertex_descriptor vertex_desc;

    rDn_3::vertex_iterator vit, vit_end;
    for (boost::tie(vit, vit_end) = boost::vertices(*this); vit != vit_end; ++vit) {
        if (vh->point() == *(*this)[*vit]) {
            vertex_desc = *vit;
            break;
        }
    }

    return vertex_desc;
}

Net_3::vertex_descriptor rDn_3::find_nearest_node(Point_3 p) const {
    Delaunay_3::Vertex_handle nearest_vertex = tessellation.nearest_vertex(p);
    return search_net_vertex(nearest_vertex);
}

Polyhedron_3 rDn_3::build_cell(const std::shared_ptr<Node_3> node_ptr) const {
    Delaunay_3::Vertex_handle vh = search_geom_vertex(node_ptr);

    std::vector<Point_3> voronoi_cell_points;
    std::vector<Point_3> open_voronoi_cell_points;

    std::vector<Delaunay_3::Cell_handle> cells; // vh を頂点に持つ四角錐
    tessellation.incident_cells(vh, std::back_inserter(cells)); 

    Polyhedron_3 voronoi_cell;
    for (const auto& cell : cells) {
        if (tessellation.is_infinite(cell)) {
            return  voronoi_cell;
        } else {
            Point_3 circumcenter = cell->circumcenter();
            voronoi_cell_points.push_back(circumcenter);
        }
    }

    CGAL::convex_hull_3(voronoi_cell_points.begin(), 
                        voronoi_cell_points.end(), 
                        voronoi_cell);

    return voronoi_cell;

}

//** File IO Method **//
void rDn_3::write_cells(const std::string& abs_file_path) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    const std::vector<std::shared_ptr<Node_3>> node_ptrs = collect_node_ptrs();


    std::string file_extension = get_file_extension(abs_file_path);
    std::ofstream file(abs_file_path);
    if (file_extension == "obj") {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            CGAL::IO::write_OBJ(file, voronoi_cell);
        }
    } else if (file_extension == "stl") {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            CGAL::IO::write_STL(file, voronoi_cell);
        }
    } else {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            file << voronoi_cell;
        }
    }

}

void rDn_3::write_cells(const std::string& abs_file_path, const std::vector<std::shared_ptr<Node_3>> node_ptrs) const {
    std::ofstream f(abs_file_path);

    if (!f.is_open()) {
        throw std::runtime_error("Fail to open " + abs_file_path + ".\n"
                                 "Error at " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    std::string file_extension = get_file_extension(abs_file_path);
    std::ofstream file(abs_file_path);
    if (file_extension == "obj") {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            CGAL::IO::write_OBJ(file, voronoi_cell);
        }
    } else if (file_extension == "stl") {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            CGAL::IO::write_STL(file, voronoi_cell);
        }
    } else {
        for (auto& node_ptr : node_ptrs) {
            Polyhedron_3 voronoi_cell = build_cell(node_ptr);
            file << voronoi_cell;
        }
    }
    
}