/*************************************************
 * @file SA.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief simulated anealing
 * @version 0.1
 * @date 2025-04-19
 * 
 * @copyright Copyright (c) 2024 Shota TABATA
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *************************************************/

#ifndef SA_H
#define SA_H

// include STL
#include <iostream>
#include <functional>
#include <random>
#include <cmath>
#include <chrono>

// include random engine
#include "core/util/random_engine.h"

template <typename Solution>
class Simulated_Annealing {
    private:

        double initial_temperature; // 初期温度
        double cooling_rate;        // 冷却率
        size_t max_iteration;       // 最大反復回数

        std::function<double(const Solution&)> evaluate_function;               // 評価関数
        std::function<Solution(const Solution&)> generate_neighbor_function;    // 近傍生成関数

        /*************************************************
         * @brief 解の遷移を受け入れるかどうかの判定
         * 
         * @param current_cost 
         * @param next_cost 
         * @param temperature 
         * @return true 
         * @return false 
         *************************************************/
        bool should_accept(double current_cost, double next_cost, double temperature) {
            if (current_cost > next_cost) {
                return true;
            } else {
                std::uniform_real_distribution<double> distribution(0.0, 1.0);
                double probability = std::exp((current_cost - next_cost) / temperature);
                return probability > distribution(Random_Engine::get_engine());            
            }

        }

        /*************************************************
         * @brief 進捗状況の表示
         * 
         * @param iteration 
         * @param max_iteration 
         * @param temperature 
         * @param current_cost 
         * @param duration 
         *************************************************/
        void log_progress(size_t iteration, size_t max_iteration, double temperature, double current_cost, double duration) const {
            {
                std::cout << "Iteration: " << iteration + 1 << " / " << max_iteration
                          << ", Temperature: " << temperature
                          << ", Cost: " << current_cost
                          << ", Duration: " << duration << " millsec / iter" << std::endl;
            }
        }

    public:

        static constexpr double MIN_TEMPERATURE = 1e-3; // 最低温度
        
        //** Constructor **//
        Simulated_Annealing(
            double initial_temperature,
            double cooling_rate,
            size_t max_iteration,
            std::function<double(const Solution&)> evaluate_function,
            std::function<Solution(const Solution&)> generate_neighbor_function
        ) : initial_temperature(initial_temperature),
            cooling_rate(cooling_rate),
            max_iteration(max_iteration),
            evaluate_function(evaluate_function),
            generate_neighbor_function(generate_neighbor_function) {}
        
        /*************************************************
         * @brief 求解
         * 
         * @param initial_solution 
         * @param show_progress 
         * @param logging 
         * @param log_file_ptr 
         * @return Solution 
         *************************************************/
        Solution solve(Solution initial_solution, 
                       bool show_progress=false, 
                       bool logging=false, 
                       std::ofstream* log_file_ptr=nullptr) {

            Solution best_solution = initial_solution;
            Solution current_solution = initial_solution;
            double best_cost = this->evaluate_function(initial_solution);
            double current_cost = best_cost;
            double temperature = this->initial_temperature;
            
            for (size_t i {0}; i < this->max_iteration && temperature > MIN_TEMPERATURE; ++i) {

                std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
                
                Solution next_solution = this->generate_neighbor_function(current_solution); 
                double next_cost = this->evaluate_function(next_solution);
                double delta_cost = next_cost - current_cost;
                
                std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                // 最適解の更新
                if (next_cost < best_cost) {
                    best_solution = next_solution;
                    best_cost = next_cost;
                }

                // 解の更新
                if (should_accept(current_cost, next_cost, temperature)) {
                    current_solution = next_solution;
                    current_cost = next_cost;
                }
                
                temperature *= this->cooling_rate;

                if (show_progress) {
                    if ((i + 1) % 100 == 0) {
                        log_progress(i, max_iteration, temperature, current_cost, duration);
                    }
                }

                if (logging) {
                    *log_file_ptr << i << " " << temperature << " " << best_cost << " " << duration << std::endl;
                }
            
            }
            
            return best_solution;
        
        }

};

#endif // SA_H