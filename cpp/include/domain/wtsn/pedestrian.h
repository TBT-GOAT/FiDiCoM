

#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

// include std libraries
#include <random>


class Pedestrian {
    private:
        double coef = 1.0;                     // 重みの感じやすさの係数（対数正規分布）
                                               // デフォルトは重み通りに感じるように設定
        static constexpr double MU_COEF = 0.0; // 重みの感じやすさの平均（正規分布）
                                               // 係数の中央値が1になるように固定
        double sigma_coef;                     // 重みの感じやすさのばらつき（正規分布）
        std::normal_distribution<double> dist; // 正規分布（指数にして対数正規分布に変換される）

        //** Simulation Functions **//
        /*************************************************
         * @brief 重みを感じる係数を変更する
         * 
         *************************************************/
        void change_coef();

    public:
        //** Constructor **//
        Pedestrian(double sigma_coef=0.5);

        //** Getter **//
        double get_coef() const;
        static double get_mu_coef();
        double get_sigma_coef() const;

        //** Setter **//
        void set_coef(const double coef);
        void set_sigma_coef(const double sigma_coef);

        //** Simulation Functions **//
        /*************************************************
         * @brief 歩行者を変更する（＝重みを変更する）
         * 
         *************************************************/
        void change_pedestrian();
};


#endif // PEDESTRIAN