/*************************************************
 * @file std_vector_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief vector utility functions
 * @version 0.1
 * @date 2024-11-20
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

#ifndef STD_VECTOR_UTIL_H
#define STD_VECTOR_UTIL_H

// include standards
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

// include random_engine
#include "core/util/random_engine.h"

/**************************************************************************//**
 * @brief 入力した要素が何番目にあるかを検索する
 * 
 * @tparam T 
 * @param vec ベクター
 * @param item 検索する要素
 * @return int 要素番号
******************************************************************************/
template <typename T>
int find_index(const std::vector<T>& vec, const T& item) {
    // 同じ要素が出てくるまで走査する
    auto ret = std::find(vec.begin(), vec.end(), item);

    // 要素が見つかった場合
    if (ret != vec.end()) {
        return ret - vec.begin();
    }
    
    // 要素が見つからなかった場合
    return -1;
}

template <typename T>
std::vector<T> random_sampling(const std::vector<T>& vec, const size_t sample_size, const bool with_replacement) {
    
    if (vec.empty()) {
        if (sample_size == 0) return {};
        throw std::invalid_argument("Cannot sample from an empty vector.");
    }

    if (!with_replacement && sample_size > vec.size()) {
        throw std::invalid_argument(
            "sample_size must be <= vec.size() when sampling without replacement."
        );
    }

    // ランダムエンジンを準備
    auto& engine = Random_Engine::get_engine();
    std::uniform_int_distribution<> dist(0, vec.size() - 1);

    // ランダムなインデックスを選択
    std::vector<T> sampled;
    sampled.reserve(sample_size);
    if (with_replacement) {
        // 重複あり
        for (size_t i = 0; i < sample_size; ++i) {
            sampled.push_back(vec[dist(engine)]);
        }
    } else {
        // 重複なし
        std::unordered_set<size_t> selected_indices;
        selected_indices.reserve(sample_size);

        while (selected_indices.size() < sample_size) {
            selected_indices.insert(dist(engine));
        }

        for (size_t index : selected_indices) {
            sampled.push_back(vec[index]);
        }
    }

    return sampled;

}

#endif // STD_VECTOR_UTIL_H