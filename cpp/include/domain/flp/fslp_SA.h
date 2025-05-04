/*************************************************
 * @file fslp_SA.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief simulated annealing for facility-sign location problem
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

#ifndef FSLP_SA_H
#define FSLP_SA_H

// include STL
#include <vector>

// include Net_FSLP
#include "domain/flp/net_fslp.h"

// include utils
#include "core/util/std_vector_util.h"

class FSLP_SA {
    public:
        static constexpr size_t MODE_MINSUM = 0;    // ミニサムモード
        static constexpr size_t MODE_MINMAX = 1;    // ミニマックスモード

        Net_FSLP net_fslp;

        //** Constructor **//
        FSLP_SA(Net_FSLP net_fslp);

        //** Objective Function Method **//
        /*************************************************
         * @brief 評価関数
         * 
         * @param facilities_and_signs 
         * @param mode 
         * @return double 
         *************************************************/
        double evaluate_function(
            const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs, 
            const size_t mode=MODE_MINSUM
        );

        //** Neighbor Generation Method **//
        /*************************************************
         * @brief 近傍生成関数（ジャンプなし）
         * 
         * @param facilities_and_signs 
         * @return std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> 
         *************************************************/
        std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> generate_neighbor_function_without_jump(
            const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs
        ) const;
        /*************************************************
         * @brief 近傍生成関数（ジャンプあり）
         * 
         * @param facilities_and_signs 
         * @param jump_rate 
         * @return std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> 
         *************************************************/
        std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> generate_neighbor_function_with_jump(
            const std::pair<std::vector<Net_2::vertex_descriptor>, std::vector<Net_2::vertex_descriptor>> facilities_and_signs, 
            const double jump_rate=0.1
        ) const;

};

#endif // FSLP_SA_H