

#ifndef DYNAMIC_WEIGHT_H
#define DYNAMIC_WEIGHT_H

// include std libraries
#include <cstddef>

// include Pedestrian
#include "domain/wtsn/pedestrian.h"

class Weight_Passability_WTSN {
    private:  
        static Pedestrian pedestrian;       // 歩行者

        bool is_fixed = false; // 変化しない重みかどうか

        static double global_init_weight;           // 重みの初期値の基準値
        double default_init_weight;                 // 重みの初期値のデフォルト値（上限）
        static constexpr double CONV_WEIGHT = 1.0;  // 重みの収束値（下限）
        double omega;                               // 重みの初期値のデフォルト値（上限）の基準値に対する比

        static double kappa; // 短距離選好度（重みを重視 0 <= kappa <= 1 距離を重視）

        double curr_count;                        // 現在のカウント
        static constexpr double DAMP_COUNT = 1.0; // カウントの減衰分（基準）
        double amp_count;                         // カウントの増幅分
        size_t amp_count_standadizer;             // カウントの増幅分の正規化（始終点の組み合わせの数）        

        size_t damp_required_count; // 減衰速度（重みが初期値から収束値に至るまでの回数）
        size_t amp_required_count;  // 増幅速度（重みが収束力から初期値に至るまでの回数）

        double sigmoid_band = 20.0; // シグモイド関数のバンド幅

        //** Simulation Functions **//
        /*************************************************
         * @brief シグモイド関数に従い重みを出力する
         * 
         * @return double 
         *************************************************/
        double sigmoid() const;
        /*************************************************
         * @brief 歩行者に応じた重みを出力する
         * 
         * @param pedestrian 
         * @return double 
         *************************************************/
        double get_weight_for(const Pedestrian pedestrian) const;

    public:
        //** Constructor **//
        Weight_Passability_WTSN();
        Weight_Passability_WTSN(const double init_weight, 
                                const size_t amp_count_standadizer, 
                                const size_t damp_required_count, 
                                const size_t amp_required_count);

        //** Getter **//
        static Pedestrian& get_pedestrian();
        bool get_is_fixed() const;
        static double get_global_init_weight();
        double get_default_init_weight() const;
        double get_omega() const;
        double get_init_weight() const;
        double get_init_weight(double _kappa) const;
        static double get_conv_weight();
        double get_curr_weight() const;
        static double get_kappa();
        double get_curr_count() const;
        static double get_damp_count();
        double get_amp_count() const;
        size_t get_amp_count_standadizer() const;
        size_t get_damp_required_count() const;
        size_t get_amp_required_count() const;
        double get_sigmoid_band() const;

        //** Setter **//
        void set_is_fixed(const bool is_fixed);
        static void set_global_init_weight(const double global_init_weight);
        void set_init_weight(const double init_weight);
        static void set_kappa(const double kappa);
        void set_curr_count(const double curr_count);
        void set_amp_count_standadizer(const size_t amp_count_standadizer);
        void set_damp_required_count(const size_t damp_required_count);
        void set_amp_required_count(const size_t amp_required_count);
        void set_sigmoid_band(const double sigmoid_band);

        //** Simulation Functions **//
        void initialize();
        double damp();
        double amp();
        double calc_weight_for_pedestrian() const;

};


#endif // DYNAMIC_WEIGHT_H