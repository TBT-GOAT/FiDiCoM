#ifndef DEMAND_SAMPLER_H
#define DEMAND_SAMPLER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <random>
#include <stdexcept>
#include <vector>

enum class Demand_Sampling_Policy { Resampled, Fixed };

template <typename Demand>
class Demand_Sampler {
    private:
        const Demand_Sampling_Policy policy;    // サンプリング方法
        std::mt19937 engine;                    // 乱数生成器
        std::vector<Demand> fixed_demands;      // 固定サンプリング用の需要点の保存場所

    public:
        using Sampling_Policy = Demand_Sampling_Policy;

        Demand_Sampler(Sampling_Policy policy, unsigned int seed)
            : policy(policy), engine(seed) {}

        /*************************************************
         * @brief サンプリングを行う関数
         * 
         * @param demands 需要点
         * @param sampling_rate サンプリング率
         * @param evaluate_all_demands 全ての需要点を評価するかどうか
         * @return std::vector<Demand> 
         *************************************************/
        std::vector<Demand> sample(const std::vector<Demand>& demands,
                                   double sampling_rate,
                                   bool evaluate_all_demands=false) {
            if (evaluate_all_demands || sampling_rate >= 1.0) {
                // 全ての需要点を評価する場合は、元の需要点をそのまま返す
                return demands;
            }

            if (!std::isfinite(sampling_rate) || sampling_rate <= 0.0) {
                // サンプリング率が正の有限値でない場合はエラー
                throw std::invalid_argument("Sampling rate must be positive and finite");
            }

            const std::size_t sample_size = static_cast<std::size_t>(demands.size() * sampling_rate);
            
            if (sample_size == 0) {
                // サンプルサイズが0の場合はエラー
                throw std::invalid_argument("Evaluation sample must contain at least one demand");
            }

            if (policy == Sampling_Policy::Fixed && fixed_demands.size() == sample_size) {
                // 固定サンプリングで、既に同じサイズのサンプルがある場合はそれを返す
                return fixed_demands;
            }

            // ランダムにサンプルを選択
            std::vector<Demand> selected_demands;
            selected_demands.reserve(sample_size);
            std::sample(demands.begin(), demands.end(),
                        std::back_inserter(selected_demands), sample_size, engine);
            
            if (policy == Sampling_Policy::Fixed) {
                // 固定サンプリングの場合は、選択したサンプルを保存
                fixed_demands = selected_demands;
            }
            
            return selected_demands;
        }
};

#endif