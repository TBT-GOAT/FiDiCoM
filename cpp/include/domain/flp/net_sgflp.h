/*************************************************
 * @file net_sgflp.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 2D netowrk for signage-guided facility location problem
 * @details BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *************************************************/

#ifndef NET_SGFLP_H
#define NET_SGFLP_H

// include STL
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>

// include Net_2
#include "core/net/net_2.h"

class Net_SGFLP {

    private:       
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_FACILITIES = -1;   // 未初期化判定用サービス供給点割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_FACILITIES;                                            // ダミーサービス供給点の点
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_SIGNS = -2;        // 未初期化判定用サイン割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_SIGNS;                                                 // ダミーサインの点
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_ANCHORS = -3;      // 未初期化判定用拠点割当用ダミー頂点インデックス
        static const Point_2 DUMMY_POINT_ANCHORS;                                               // ダミー拠点の点
        static constexpr Net_2::vertex_descriptor UNINITIALIZED_DUMMY_VERTEX_ENTITIES = -4;     // 未初期化判定用エンティティダミー頂点インデックス

        Point_2 inner_point;                                        // 需要のある空間の内部点
        double facility_visible_length = Net_SGFLP::VISIBLE_LENGTH; // サービス供給点の可視長さ
        double sign_visible_length = Net_SGFLP::VISIBLE_LENGTH;     // サインの可視長さ
        double anchor_visible_length = Net_SGFLP::VISIBLE_LENGTH;   // 拠点の可視長さ
        
        std::vector<Net_2::vertex_descriptor> demands {};       // 需要点
        std::vector<Net_2::vertex_descriptor> facilities {};    // サービス供給点
        std::vector<Net_2::vertex_descriptor> signs {};         // サイン（視認できたとき最寄りのサービス供給点を指示する）
        std::vector<Net_2::vertex_descriptor> anchors {};       // 拠点（場所が周知の特別なサービス供給点。ユーザーがまず最初に向かう場所）
        
        Net_2::vertex_descriptor dummy_vertex_facilities = UNINITIALIZED_DUMMY_VERTEX_FACILITIES;   // サービス供給点割当用ダミー頂点
        Net_2::vertex_descriptor dummy_vertex_signs = UNINITIALIZED_DUMMY_VERTEX_SIGNS;             // サイン割当用ダミー頂点
        Net_2::vertex_descriptor dummy_vertex_anchors = UNINITIALIZED_DUMMY_VERTEX_ANCHORS;         // 拠点割当用ダミー頂点
        
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > facility_coverage_trees {};                                                               // 可視であるサービス供給点を示す最短経路木
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > facility_shortest_path_trees {};                                                          // サービス供給点までの最短経路計算用
        std::vector<std::pair<Net_2::vertex_descriptor, double>> facility_shortest_path_tree {};    // 最寄りのサービス供給点割当用最短経路木
        
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > sign_coverage_trees {};                                                                   // 可視であるサインを示す最短経路木
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > sign_shortest_path_trees {};                                                              // サインまでの最短経路計算用
        std::vector<std::pair<Net_2::vertex_descriptor, double>> sign_shortest_path_tree {};        // 最寄りのサイン割当用最短経路木
        
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > anchor_coverage_trees {};                                                                 // 可視である拠点を示す最短経路木                                   
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > anchor_shortest_path_trees {};                                                            // 拠点までの最短経路計算用
        std::vector<std::pair<Net_2::vertex_descriptor, double>> anchor_shortest_path_tree {};      // 最寄りの拠点割当用最短経路木
        
        // 割当＝可視かつ最寄り
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> facility_assignment_to_demand;   // 需要点に対するサービス供給点の割当（キー：需要点，値：サービス供給点）
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> sign_assignment_to_demand;       // 需要点に対するサインの割当（キー：需要点，値：サイン）
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> anchor_assignment_to_demand;     // 需要点に対する拠点の割当（キー：需要点，値：拠点）
        // 経路割当
        std::vector<std::vector<Net_2::vertex_descriptor>> entity_groups;                               // サービス供給点、サイン、拠点をまとめたもの（可視距離行列のインデックスに対応）
        std::vector<std::vector<double>> visibility_distance_matrix;                                    // 可視距離行列
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> navigation_assignment;   // サービス供給点、または、拠点に至るまでの経路割当（キー：サイン，値：サービス供給点、サイン、または、拠点）

        //** Network Methods **//
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_facilities();
        /*************************************************
         * @brief サインを束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_signs();
        /*************************************************
         * @brief 拠点を束ねるダミー頂点を挿入する
         * 
         *************************************************/
        void insert_dummy_vertex_anchors();
        /*************************************************
         * @brief サービス供給点を束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_facilities();
        /*************************************************
         * @brief サインを束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_signs();
        /*************************************************
         * @brief 拠点を束ねるダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertex_anchors();
        /*************************************************
         * @brief サービス供給点から可視である範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_coverage_trees();
        /*************************************************
         * @brief サービス供給点までの最短経路を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_shortest_path_trees();
        /*************************************************
         * @brief サービス供給点から最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_facility_shortest_path_tree();
        /*************************************************
         * @brief サインから可視である範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_sign_coverage_trees();
        /*************************************************
         * @brief サインまでの最短経路を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_sign_shortest_path_trees();
        /*************************************************
         * @brief サインから最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_sign_shortest_path_tree();
        /*************************************************
         * @brief 拠点から可視である範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_anchor_coverage_trees();
        /*************************************************
         * @brief 拠点までの最短経路を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_anchor_shortest_path_trees();
        /*************************************************
         * @brief 拠点から最寄りの範囲を計算するための最短経路木を構築する
         * 
         *************************************************/
        void build_anchor_shortest_path_tree();
        /*************************************************
         * @brief 最寄りの拠点までの経路を計算する
         * 
         * @param demand 
         * @return std::deque<Net_2::vertex_descriptor> 
         *************************************************/
        std::deque<Net_2::vertex_descriptor> calculate_path_to_anchor(Net_2::vertex_descriptor demand) const;

        //** Initialization Methods **//
        /*************************************************
         * @brief 指定した点に最も近い需要点を探索する
         * 
         * @param points 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> search_nearest_demands(const std::vector<Point_2> points) const;
        /*************************************************
         * @brief 需要点をランダムに選択する
         * 
         * @param selected_num 
         * @return std::vector<Net_2::vertex_descriptor> 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> select_demands_at_random(const size_t selected_num);
        /*************************************************
         * @brief エンティティ（サービス供給点、サイン、拠点）を初期化する
         * 
         *************************************************/
        void initialize_entity_groups();
        /*************************************************
         * @brief 可視距離行列を初期化する
         * 
         *************************************************/
        void initialize_visibility_distance_matrix();
        
        //** Constraint Method **//
        /*************************************************
         * @brief 需要点に可視である最寄りのサービス供給点を割り当てる
         * 
         *************************************************/
        void assign_facility_to_demand();
        /*************************************************
         * @brief 需要点に可視である最寄りのサインを割り当てる
         * 
         *************************************************/
        void assign_sign_to_demand();
        /*************************************************
         * @brief 需要点に可視である最寄りの拠点を割り当てる
         * 
         *************************************************/
        void assign_anchor_to_demand();
        /*************************************************
         * @brief 経路割当を実行する
         * 
         *************************************************/
        void assign_navigation();

        // /*************************************************
        //  * @brief サインに最寄りのサービス供給点を割り当てる
        //  * 
        //  *************************************************/
        // void assign_facility_to_sign();
        // /*************************************************
        //  * @brief 拠点に最寄りのサービス供給点を割り当てる
        //  * 
        //  *************************************************/
        // void assign_facility_to_anchor();
        // /*************************************************
        //  * @brief 需要点に最寄りの拠点を割り当てる
        //  * 
        //  *************************************************/
        // void assign_anchor_to_demand();
        // /*************************************************
        //  * @brief 需要点に対するサービス供給点の割当を更新する
        //  * 
        //  *************************************************/
        // void update_facility_assignment_to_demand();

    public:
        static constexpr double VISIBLE_LENGTH = 1.0e9; // 可視の長さのデフォルト値（1000 km）
        
        std::shared_ptr<Net_2> net_ptr; // 任意のNet_2のポインタ

        // コストのパターン
        // 需要点がいずれかの可視範囲にある場合
        static const size_t COST_PATTERN_DinA;  // 0: 需要点 --> 拠点
        static const size_t COST_PATTERN_DinF;  // 1: 需要点 --> サービス供給点
        static const size_t COST_PATTERN_DinS;  // 2: 需要点 --> サイン --> サービス供給点 or 拠点
        // 需要点がいずれの可視範囲にもない場合
        static const size_t COST_PATTERN_DoutA; // 3: 需要点 --> 拠点
        static const size_t COST_PATTERN_DoutF; // 4: 需要点 --> サービス供給点
        static const size_t COST_PATTERN_DoutS; // 5: 需要点 --> サイン --> サービス供給点 or 拠点
        static const size_t COST_PATTERN_Duncovered; // 6: 需要点 --> 拠点

        //** Constructor **//
        Net_SGFLP();
        Net_SGFLP(std::shared_ptr<Net_2> net_ptr);

        //** Getter **//
        Point_2 get_inner_point() const;

        double get_facility_visible_length() const;
        double get_sign_visible_length() const;
        double get_anchor_visible_length() const;

        std::vector<Net_2::vertex_descriptor> get_demands() const;
        std::vector<Net_2::vertex_descriptor> get_facilities() const;
        std::vector<Net_2::vertex_descriptor> get_signs() const;
        std::vector<Net_2::vertex_descriptor> get_anchors() const;

        Net_2::vertex_descriptor get_dummy_vertex_facilities() const;
        Net_2::vertex_descriptor get_dummy_vertex_signs() const;
        Net_2::vertex_descriptor get_dummy_vertex_anchors() const;

        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > get_facility_coverage_trees() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_facility_shortest_path_tree() const;
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > get_sign_coverage_trees() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_sign_shortest_path_tree() const;
        std::unordered_map<
            Net_2::vertex_descriptor, 
            std::vector<std::pair<Net_2::vertex_descriptor, double>>
        > get_anchor_coverage_trees() const;
        std::vector<std::pair<Net_2::vertex_descriptor, double>> get_anchor_shortest_path_tree() const;

        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_facility_assignment_to_demand() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_sign_assignment_to_demand() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_anchor_assignment_to_demand() const;

        std::vector<std::vector<Net_2::vertex_descriptor>> get_entity_groups() const;
        std::vector<std::vector<double>> get_visibility_distance_matrix() const;
        std::unordered_map<Net_2::vertex_descriptor, Net_2::vertex_descriptor> get_navigation_assignment() const;

        //** Setter **//
        void set_inner_point(const Point_2 inner_point);

        void set_facility_visible_length(const double visible_length);
        void set_sign_visible_length(const double visible_length);
        void set_anchor_visible_length(const double visible_length);

        void set_demands();
        void set_demands(const Point_2 inner_point);
        void set_demands(const std::vector<Net_2::vertex_descriptor> demands);
        void set_facilities(const std::vector<Net_2::vertex_descriptor> facilities);
        void set_signs(const std::vector<Net_2::vertex_descriptor> signs);
        void set_anchors(const std::vector<Net_2::vertex_descriptor> anchors);

        void set_dummy_vertex_facilities(const Net_2::vertex_descriptor dummy_vertex_facilities);
        void set_dummy_vertex_signs(const Net_2::vertex_descriptor dummy_vertex_signs);
        void set_dummy_vertex_anchors(const Net_2::vertex_descriptor dummy_vertex_anchors);

        //** Network Methods **//
        /*************************************************
         * @brief すべてのダミー頂点を削除する
         * 
         *************************************************/
        void remove_dummy_vertices();
        /*************************************************
         * @brief ダミー頂点が束ねるサービス供給点を更新する
         * 
         *************************************************/
        void update_dummy_vertex_facilities(const Net_2::vertex_descriptor prev_facility_vertex, 
                                            const Net_2::vertex_descriptor next_facility_vertex);
        /*************************************************
         * @brief ダミー頂点が束ねるサインを更新する
         * 
         * @param prev_sign_vertex 
         * @param next_sign_vertex 
         *************************************************/
        void update_dummy_vertex_signs(const Net_2::vertex_descriptor prev_sign_vertex, 
                                       const Net_2::vertex_descriptor next_sign_vertex);
        /*************************************************
         * @brief 各種の割当用の最短経路木を構築する
         * 
         *************************************************/
        void build_trees();
        /*************************************************
         * @brief 各種の割当用の最短経路木を削除する
         * 
         *************************************************/
        void clear_trees();

        //** Initialization Methods **//
        /*************************************************
         * @brief 最寄りの需要点を探索する
         * 
         * @param point 
         * @return Net_2::vertex_descriptor 
         *************************************************/
        Net_2::vertex_descriptor search_nearest_demand(const Point_2 point) const;
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param demand_points 
         *************************************************/
        void initialize_facilities(const std::vector<Point_2> facility_points);
        /*************************************************
         * @brief サービス供給点を初期化する
         * 
         * @param facility_num 
         *************************************************/
        void initialize_facilities(const size_t facility_num);
        /*************************************************
         * @brief サインを初期化する
         * 
         * @param sign_points 
         *************************************************/
        void initialize_signs(const std::vector<Point_2> sign_points);
        /*************************************************
         * @brief サインを初期化する
         * 
         * @param sign_num 
         *************************************************/
        void initialize_signs(const size_t sign_num);
        /*************************************************
         * @brief 拠点を初期化する
         * 
         * @param anchor_points 
         *************************************************/
        void initialize_anchors(const std::vector<Point_2> anchor_points);
        /*************************************************
         * @brief 経路割当を初期化する
         * 
         *************************************************/
        void initialize_navigation_assignment();

        //** Clear Methods **//
        /*************************************************
         * @brief 設定をクリアする（すべてのダミー頂点を削除する）
         * 
         *************************************************/
        void clear();

        //** Constraint Method **//
        /*************************************************
         * @brief 需要点に対するサービス供給点の割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_facility_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 需要点に対するサインの割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_sign_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 需要点に対する拠点の割当を取得する
         * 
         * @param demand 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_anchor_to_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 経路割当上で割り当てられた次のエンティティを取得する
         * 
         * @param entity 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> get_assigned_entity_on_navigation(Net_2::vertex_descriptor entity) const;
        /*************************************************
         * @brief 各種の割当を実行する
         * 
         *************************************************/
        void build_assignments();
        /*************************************************
         * @brief 割当をクリアする
         * 
         *************************************************/
        void clear_assignments();

        //** Cost Function Methods **//
        /*************************************************
         * @brief 需要点のコストを計算する
         * TODO 面積の考慮
         * 
         * @param demand 
         * @return std::pair<size_t, double> コストのパターン, コスト 
         *************************************************/
        std::pair<size_t, double> calculate_cost(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 見えている最寄りのサービス供給点に向かうときのコストを計算する
         * 
         * @param demand 
         * @return std::pair<size_t, double> コストのパターン, コスト 
         *************************************************/
        std::pair<size_t, double> calculate_cost_to_visible_facility(Net_2::vertex_descriptor demand, 
                                                                     Net_2::vertex_descriptor facility) const;
        /*************************************************
         * @brief サインに従うときのコストを計算する
         * 
         * @param demand 
         * @return std::pair<size_t, double> コストのパターン, コスト 
         *************************************************/
        std::pair<size_t, double> calculate_cost_to_follow_signage(Net_2::vertex_descriptor demand, 
                                                                   Net_2::vertex_descriptor sign) const;
        /*************************************************
         * @brief 見えている最寄りの拠点に向かうときのコストを計算する
         * 
         * @param demand 
         * @return std::pair<size_t, double> コストのパターン, コスト 
         *************************************************/
        std::pair<size_t, double> calculate_cost_to_visible_anchor(Net_2::vertex_descriptor demand, 
                                                                   Net_2::vertex_descriptor anchor) const;
        /*************************************************
         * @brief どのエンティティも不可視の需要点からのコストを計算する
         * 
         * @param demand 
         * @return std::pair<size_t, double> 
         *************************************************/
        std::pair<size_t, double> calculate_cost_from_uncovered_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 経路（o --> s --> t --> d）上の2頂点間のコストを計算する
         * 
         * @param s 
         * @param t 
         * @param path 
         * @return double 
         *************************************************/
        double calculate_cost_on_path(Net_2::vertex_descriptor s,
                                      Net_2::vertex_descriptor t,
                                      const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const;
        /*************************************************
         * @brief 経路（origin --> destination）上の中継地点（エンティティ（destination）に割当済の頂点）を探索する
         * @details origin は path の先頭の頂点、destination は path の途中の頂点であることを想定している 
         * @param destination
         * @param path 
         * @return std::pair<bool, Net_2::vertex_descriptor> 
         *************************************************/
        std::pair<bool, Net_2::vertex_descriptor> find_waystop(Net_2::vertex_descriptor destination,
                                                               const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const;
        /*************************************************
         * @brief 経路上で最初に認識できるエンティティを探索する
         * 
         * @param path 
         * @return std::tuple<int, Net_2::vertex_descriptor,Net_2::vertex_descriptor> 
         * 0: 拠点、1: サービス供給点、2: サイン、-1: どのエンティティも認識できない
         * 最初のNet_2::vertex_descriptorは最初にエンティティを認識する頂点
         * 次のNet_2::vertex_descriptorはその頂点が割り当てられたエンティティ
         *************************************************/
        std::tuple<int, 
                   Net_2::vertex_descriptor, 
                   Net_2::vertex_descriptor> find_first_entity(const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const;

        //** Path Calculation Methods **//
        /*************************************************
         * @brief 需要点に対する経路を計算する
         * 
         * @param demand 
         * @return std::pair<size_t, std::vector<Net_2::vertex_descriptor>> コストのパターン, 経路 
         *************************************************/
        std::pair<size_t, std::vector<Net_2::vertex_descriptor>> calculate_path(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 見えている最寄りのサービス供給点に向かうときの経路を計算する
         * 
         * @param demand 
         * @return std::pair<size_t, std::vector<Net_2::vertex_descriptor>> コストのパターン, 経路 
         *************************************************/
        std::pair<size_t, std::vector<Net_2::vertex_descriptor>> calculate_path_to_visible_facility(Net_2::vertex_descriptor demand, 
                                                                                                     Net_2::vertex_descriptor facility) const;
        /*************************************************
         * @brief サインに従うときの経路を計算する
         * 
         * @param demand 
         * @return std::pair<size_t, std::vector<Net_2::vertex_descriptor>> コストのパターン, 経路 
         *************************************************/
        std::pair<size_t, std::vector<Net_2::vertex_descriptor>> calculate_path_to_follow_signage(Net_2::vertex_descriptor demand, 
                                                                                                   Net_2::vertex_descriptor sign) const;
        /*************************************************
         * @brief 見えている最寄りの拠点に向かうときの経路を計算する
         * 
         * @param demand 
         * @return std::pair<size_t, std::vector<Net_2::vertex_descriptor>> コストのパターン, 経路 
         *************************************************/
        std::pair<size_t, std::vector<Net_2::vertex_descriptor>> calculate_path_to_visible_anchor(Net_2::vertex_descriptor demand, 
                                                                                                   Net_2::vertex_descriptor anchor) const;
        /*************************************************
         * @brief どのエンティティも不可視の需要点からの経路を計算する
         * 
         * @param demand 
         * @return std::pair<size_t, std::vector<Net_2::vertex_descriptor>> コストのパターン, 経路 
         *************************************************/
        std::pair<size_t, std::vector<Net_2::vertex_descriptor>> calculate_path_from_uncovered_demand(Net_2::vertex_descriptor demand) const;
        /*************************************************
         * @brief 経路（o --> s --> t --> d）上の2頂点間の経路を計算する
         * 
         * @param s 
         * @param t 
         * @param path 
         * @return std::vector<Net_2::vertex_descriptor>                                 
         *************************************************/
        std::vector<Net_2::vertex_descriptor> calculate_path_between(Net_2::vertex_descriptor s,
                                                                     Net_2::vertex_descriptor t,
                                                                     const std::deque<std::pair<Net_2::vertex_descriptor, double>>& path) const;


};

#endif // Net_SGFLP_H