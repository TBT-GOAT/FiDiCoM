

#ifndef SIMULATOR_WTSN_H
#define SIMULATOR_WTSN_H

// include std libraries
#include <vector>
#include <tuple>

// include util
#include "core/util/random_engine.h"

// include WTSN
#include "domain/wtsn/pedestrian.h"
#include "domain/wtsn/rDn_2_wtsn.h"
#include "domain/wtsn/edge_2_wtsn.h"

class Simulator_WTSN {
    private:
        Polygon_2 domain;                            // 対象領域
        std::vector<Polygon_2> holes;                // 対象領域の穴（建物など）
                                                     //TODO CGAL Polygon_with_holes_2を使う
        std::shared_ptr<rDn_2_WTSN> net_ptr;         // ネットワーク
        std::vector<Point_2> demand_points;          // 移動需要の入力点
        std::vector<rDn_2_WTSN::vertex_descriptor> demand_nodes;    // 移動の需要点
        std::vector<std::vector<double>> trans_prob_matrix;         // 移動確率行列
        std::uniform_real_distribution<> dist{0.0, 1.0};            // [0, 1]の一様分布

        std::set<rDn_2_WTSN::edge_descriptor> unsaturated_edges; // 重みに変化があるエッジの集合
        std::set<rDn_2_WTSN::edge_descriptor> activated_edges;   // 活性化したエッジの集合
        std::set<rDn_2_WTSN::edge_descriptor> living_edges;      // 生きているエッジの集合

        // 収束条件
        size_t threshold_duration; // ネットワークが変化しない期間の閾値
        size_t max_iteration;      // 最大反復回数

        // 履歴
        std::vector<std::tuple
        <
            size_t, // 反復回数
            bool,   // 接続しているか
            bool,   // 収束しているか
            size_t, // 重みに変化があるエッジの数
            size_t, // 活性化しているエッジの数
            size_t, // 生きているエッジの数
            double  // 生きているエッジの長さの総和
        >
        > history;

        // 乱数
        size_t seed;

        //** Constructor Support Functions **//
        /*************************************************
         * @brief 対象領域内に点があるか判定する
         * TODO 対象領域に穴がある場合の対応
         * 
         * @param p 
         * @return true 
         * @return false 
         *************************************************/
        bool is_in_domain(const Point_2& p) const;
        /*************************************************
         * @brief 需要点に対応するノードを探索する
         * 
         * @param demand_points 
         * @return std::vector<rDn_2_WTSN::vertex_descriptor> 
         *************************************************/
        void find_demand_nodes(std::vector<Point_2> demand_points);
        /*************************************************
         * @brief 等確率の移動確率行列を計算する
         * 
         * @param demand_num 
         * @return std::vector<std::vector<double>> 
         *************************************************/
        std::vector<std::vector<double>> calc_uniform_trans_prob_matrix(size_t demand_num) const;
        
        //** Simulation Functions **//
        /*************************************************
         * @brief 行列を標準化する（和を1にする）
         * 
         * @param matrix 
         * @return std::vector<std::vector<double>> 
         *************************************************/
        std::vector<std::vector<double>> normalize_matrix(const std::vector<std::vector<double>>& matrix);
        /*************************************************
         * @brief 始点と終点を選択する
         * 
         * @return std::pair<rDn_2_WTSN::vertex_descriptor, rDn_2_WTSN::vertex_descriptor> 
         *************************************************/
        std::pair<rDn_2_WTSN::vertex_descriptor, rDn_2_WTSN::vertex_descriptor> choose_OD();
        /*************************************************
         * @brief 歩行者の移動軌跡（重み付き最短路）を計算する
         * 
         * @param O 
         * @param D 
         * @return std::deque<rDn_2_WTSN::vertex_descriptor> 
         *************************************************/
        std::deque<rDn_2_WTSN::vertex_descriptor> calc_pedestrian_path(const rDn_2_WTSN::vertex_descriptor O, 
                                                                       const rDn_2_WTSN::vertex_descriptor D) const;
        /*************************************************
         * @brief エッジの重みを減衰させる
         * 
         * @param pedestrian_path 
         * @param unsaturated_edges 
         *************************************************/
        void damp(std::deque<rDn_2_WTSN::vertex_descriptor> pedestrian_path);
        /*************************************************
         * @brief エッジの重みを増幅させる
         * 
         *************************************************/
        void amp();
        /*************************************************
         * @brief 各種のエッジの集合を更新する
         * 
         *************************************************/
        void update_edge_sets();
        /*************************************************
         * @brief 活性化したエッジがすべての需要点を連結しているか判定する
         * 
         * @return true 
         * @return false 
         *************************************************/
        bool is_connecting_all_demand_nodes() const;

        //** Analysis Functions **//
        /*************************************************
         * @brief 生きているエッジを使ってネットワークを構築する
         * 
         * @return Net_2 
         *************************************************/
        Net_2 build_wtsn_graph() const;

    public:
        //** Constructor **//
        Simulator_WTSN(const Polygon_2 domain, 
                       const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                       const std::vector<Point_2> demand_points, 
                       const std::vector<std::vector<double>> demand_matrix={},
                       const std::vector<Polygon_2> holes={});
        //TODO domainもinput_fileから読み込む
        Simulator_WTSN(const Polygon_2 domain, 
                       const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                       std::ifstream& input_file);
        Simulator_WTSN(const Polygon_2 domain, 
                       const std::shared_ptr<rDn_2_WTSN> net_ptr, 
                       const std::string& input_file_path);

        //** Getter **//
        std::shared_ptr<rDn_2_WTSN> get_net_ptr() const; 
        std::vector<Point_2> get_demand_points() const; 
        std::vector<rDn_2_WTSN::vertex_descriptor> get_demand_nodes() const; 
        std::vector<std::vector<double>> get_trans_prob_matrix() const; 
        std::set<rDn_2_WTSN::edge_descriptor> get_unsaturated_edges() const;
        std::set<rDn_2_WTSN::edge_descriptor> get_activated_edges() const;
        std::set<rDn_2_WTSN::edge_descriptor> get_living_edges() const;

        //** Setter **//

        //** Simulation Functions **//
        /*************************************************
         * @brief シミュレーションを設定する
         * 
         *************************************************/
        void setup(const size_t threshold_duration, 
                   const double max_iteration); 
        /*************************************************
         * @brief シミュレーションを実行する
         * 
         *************************************************/
        void run();
        /*************************************************
         * @brief 有効な結果が得られているか判定する
         * 
         * @return true 
         * @return false 
         *************************************************/
        bool has_valid_result() const;

        //** Analysis Functions **//
        /*************************************************
         * @brief WTSNから冗長なエッジを除去したネットワークを構築する
         * 
         * @return Net_2 
         *************************************************/
        Net_2 simplify_wtsn() const;
        double calc_total_length(Net_2& result_network, 
                                 bool is_weighted=false) const;
        double calc_total_detour(Net_2& result_network, 
                                 bool is_weighted=false, 
                                 std::vector<Net_2::vertex_descriptor> demand_vertices={}) const;
        std::pair<double, double> evaluate_network(Net_2& result_network, 
                                                bool is_weighted_length=false, 
                                                bool is_weighted_detour=false, 
                                                std::vector<Net_2::vertex_descriptor> demand_vertices={}, 
                                                bool checking_redundancy=false) const;
        
        //** Record Functions **//
        /*************************************************
         * @brief シミュレーションの設定を記録する
         * 
         * @param config_file 
         *************************************************/
        void save_config(std::ofstream& config_file) const;
        /*************************************************
         * @brief シミュレーションのログを記録する
         * 
         * @param log_file 
         *************************************************/
        void save_log(std::ofstream& log_file) const;

};

#endif // SIMULATOR_WTSN_H