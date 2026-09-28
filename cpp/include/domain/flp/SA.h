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
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

// include random engine
#include "core/util/random_engine.h"

struct Simulated_Annealing_Parameters {
    double initial_temperature;
    double cooling_rate;
    size_t max_iteration;
    size_t min_iteration {0};
    size_t max_no_improvement {0};
};

template <typename Solution>
class Simulated_Annealing {
    private:

        Simulated_Annealing_Parameters parameters;

        std::function<double(const Solution&)> evaluate_function;               // 評価関数
        std::function<Solution(const Solution&)> generate_neighbor_function;    // 近傍解生成関数
                                                                                //!目的関数の評価、近傍解生成は、解の生成と切り離して実装するべき。
                                                                                //!内部状態を変更するような副作用を持つ実装は避けるべき。

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

        template <typename T>
        static typename std::enable_if<std::is_floating_point<T>::value>::type log_solution_value(std::ostream& log_file, const T& value) {
            std::streamsize previous_precision = log_file.precision();
            std::ios_base::fmtflags previous_flags = log_file.flags();

            log_file << std::defaultfloat << std::setprecision(std::numeric_limits<T>::max_digits10) << value;

            log_file.precision(previous_precision);
            log_file.flags(previous_flags);
        }

        template <typename T>
        static typename std::enable_if<!std::is_floating_point<T>::value>::type log_solution_value(std::ostream& log_file, const T& value) {
            log_file << value;
        }

        template <typename T>
        static void log_solution_value(std::ostream& log_file, const std::vector<T>& values) {
            log_file << "[";
            for (size_t i {0}; i < values.size(); ++i) {
                if (i > 0) {
                    log_file << ",";
                }
                log_solution_value(log_file, values.at(i));
            }
            log_file << "]";
        }

        template <typename T, typename U>
        static void log_solution_value(std::ostream& log_file, const std::pair<T, U>& value) {
            log_file << "(";
            log_solution_value(log_file, value.first);
            log_file << ",";
            log_solution_value(log_file, value.second);
            log_file << ")";
        }

    public:

        static constexpr double MIN_TEMPERATURE = 1e-3; // 最低温度
        
        //** Constructor **//
        Simulated_Annealing(
            Simulated_Annealing_Parameters parameters,
            std::function<double(const Solution&)> evaluate_function,
            std::function<Solution(const Solution&)> generate_neighbor_function
        ) : parameters(parameters),
            evaluate_function(evaluate_function),
            generate_neighbor_function(generate_neighbor_function) {
            if (this->parameters.min_iteration > this->parameters.max_iteration) {
                throw std::invalid_argument("min_iteration must not exceed max_iteration");
            }
        }

        // 旧バージョンのコンストラクタ
        Simulated_Annealing(
            double initial_temperature,
            double cooling_rate,
            size_t max_iteration,
            std::function<double(const Solution&)> evaluate_function,
            std::function<Solution(const Solution&)> generate_neighbor_function,
            size_t min_iteration = 0,
            size_t max_no_improvement = 0
        ) : Simulated_Annealing(
                Simulated_Annealing_Parameters{
                    initial_temperature,
                    cooling_rate,
                    max_iteration,
                    min_iteration,
                    max_no_improvement
                },
                evaluate_function,
                generate_neighbor_function) {}
        
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
            double temperature = this->parameters.initial_temperature;
            size_t iterations_since_improvement {0};
            
            for (size_t i {0}; i < this->parameters.max_iteration; ++i) {
                // 終了条件の確認
                if (i >= this->parameters.min_iteration) {
                    if (temperature <= MIN_TEMPERATURE) {
                        // 温度が最小値以下になったか
                        break;
                    }
                    if (this->parameters.max_no_improvement > 0 &&
                        iterations_since_improvement >= this->parameters.max_no_improvement) {
                        // 最大改善なし回数に達したか
                        break;
                    }
                }
                // std::cout << "Iteration: " << i + 1 << " / " << this->parameters.max_iteration << std::endl;

                std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
                
                Solution next_solution = this->generate_neighbor_function(current_solution); 
                double next_cost = this->evaluate_function(next_solution);
                double delta_cost = next_cost - current_cost;
                
                std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                // 最適解の更新
                if (next_cost < best_cost) {
                    // std::cout << "New best solution found! Cost improved from " << best_cost << " to " << next_cost << std::endl;
                    best_solution = next_solution;
                    best_cost = next_cost;
                    iterations_since_improvement = 0; // 改善があったのでカウンタをリセット
                } else {
                    // 改善がなかったのでカウンタをインクリメント
                    ++iterations_since_improvement;
                }

                // 解の更新
                if (should_accept(current_cost, next_cost, temperature)) {
                    // std::cout << "Accepted new solution with cost " << next_cost << " (current cost: " << current_cost << ", temperature: " << temperature << ")" << std::endl;
                    current_solution = next_solution;
                    current_cost = next_cost;
                } else {
                    // std::cout << "Rejected new solution with cost " << next_cost << " (current cost: " << current_cost << ", temperature: " << temperature << ")" << std::endl;
                }
                
                temperature *= this->parameters.cooling_rate;

                if (show_progress) {
                    if ((i + 1) % 100 == 0) {
                        log_progress(i, this->parameters.max_iteration, temperature, current_cost, duration);
                    }
                }

                if (logging) {
                    *log_file_ptr 
                    << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10)
                    << i << " " << temperature << " " << best_cost << " " << current_cost << " " << duration
                    << " best_solution=";
                    log_solution_value(*log_file_ptr, best_solution);
                    *log_file_ptr << " current_solution=";
                    log_solution_value(*log_file_ptr, current_solution);
                    *log_file_ptr << std::endl;
                }
            
            }
            
            return best_solution;
        
        }

};

#endif // SA_H