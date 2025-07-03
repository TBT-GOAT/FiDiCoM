
// include header
#include "domain/wtsn/rDn_2_wtsn.h"

// include Edge_2_WTSN
#include "domain/wtsn/edge_2_wtsn.h"


void rDn_2_WTSN::initialize() {
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

        std::shared_ptr<Edge_2_WTSN> edge_ptr = std::make_shared<Edge_2_WTSN>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }

}

void rDn_2_WTSN::initialize(const std::vector<std::shared_ptr<Node_2>> node_ptrs) {
    
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

        std::shared_ptr<Edge_2_WTSN> edge_ptr = std::make_shared<Edge_2_WTSN>((*this)[vertex1], (*this)[vertex2]);
        boost::add_edge(vertex1, vertex2, edge_ptr, *this);
    }

}

void rDn_2_WTSN::weight_edges(std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs) {
    rDn_2_WTSN::edge_iterator eit, eit_end;
    //TODO エッジの数 × 領域の数 だけ計算している．OpenCLを使って省力化したい
    for (boost::tie(eit, eit_end) = boost::edges(*this); eit != eit_end; ++eit) {
        for (auto region_ptr : region_ptrs) {
            if (region_ptr->is_intersecting(*(*this)[*eit])) {
                // 重いものを優先
                if (std::dynamic_pointer_cast<Edge_2_WTSN>((*this)[*eit])->access_weight_passability_wtsn().get_init_weight(0.0) < region_ptr->get_weight_passability()) {
                    std::dynamic_pointer_cast<Edge_2_WTSN>((*this)[*eit])->assign_weight_passability_wtsn(region_ptr->get_weight_passability_wtsn());
                }
            } else {
                std::dynamic_pointer_cast<Edge_2_WTSN>((*this)[*eit])->assign_weight_passability_wtsn();
            }
        }
    }
}

void rDn_2_WTSN::reset_weight() {
    rDn_2_WTSN::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = boost::edges(*this); eit != eit_end; ++eit) {
        std::dynamic_pointer_cast<Edge_2_WTSN>((*this)[*eit])->access_weight_passability_wtsn().set_curr_count(0.0);
    }
}