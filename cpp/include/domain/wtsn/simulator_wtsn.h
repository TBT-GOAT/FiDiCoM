

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
    public: // TODO 消去
        Polygon_2 domain;                            // 対象領域
        std::shared_ptr<rDn_2_WTSN> net_ptr;         // ネットワーク
        std::vector<rDn_2_WTSN::vertex_descriptor> demand_nodes;    // 移動の需要点
        std::vector<std::vector<double>> trans_prob_matrix;         // 移動確率行列
        std::uniform_real_distribution<> dist{0.0, 1.0};            // [0, 1]の一様分布

        std::set<rDn_2_WTSN::edge_descriptor> unsaturated_edges; // 重みに変化があるエッジの集合
        std::set<rDn_2_WTSN::edge_descriptor> activated_edges;   // 活性化したエッジの集合

        // 収束条件
        size_t threshold_duration; // ネットワークが変化しない期間の閾値
        size_t max_iteration;      // 最大反復回数

        // ログ
        std::vector<std::tuple<size_t, bool, double>> history; // 反復回数, 収束しているか, 活性化したエッジの長さの総和

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
         * @brief 活性化したエッジの集合を更新する
         * 
         *************************************************/
        void update_activated_edges();
        /*************************************************
         * @brief 活性化したエッジがすべての需要点を連結しているか判定する
         * 
         * @return true 
         * @return false 
         *************************************************/
        bool is_connecting_all_demand_nodes();


    public:
        //** Constructor **//
        Simulator_WTSN(Polygon_2 domain, 
                       std::shared_ptr<rDn_2_WTSN> net_ptr, 
                       std::vector<Point_2> demand_points, 
                       std::vector<std::vector<double>> demand_matrix);

        //** Getter **//
        std::shared_ptr<Pedestrian> get_pedestrian_ptr() const; 
        std::shared_ptr<rDn_2_WTSN> get_net_ptr() const; 
        std::vector<rDn_2_WTSN::vertex_descriptor> get_demand_nodes() const; 
        std::vector<std::vector<double>> get_trans_prob_matrix() const; 
        std::set<rDn_2_WTSN::edge_descriptor> get_unsaturated_edges() const;
        std::set<rDn_2_WTSN::edge_descriptor> get_activated_edges() const;

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

        //** Logger **//
        //TODO

};

#endif // SIMULATOR_WTSN_H