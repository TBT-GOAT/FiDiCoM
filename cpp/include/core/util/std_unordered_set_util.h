/*************************************************
 * @file std_unordered_set_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief unordered set utility functions
 * @version 0.1
 * @date 2025-01-07
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

#ifndef STD_UNORDERED_SET_UTIL_H
#define STD_UNORDERED_SET_UTIL_H

// include standards
#include <iostream>
#include <unordered_set>
#include <algorithm>

/*************************************************
 * @brief 集合の積を計算する
 * 
 * @tparam T 
 * @param set1 
 * @param set2 
 * @return std::unordered_set<T> 
 *************************************************/
template <typename T>
std::unordered_set<T> set_intersection(const std::unordered_set<T>& set1, const std::unordered_set<T>& set2) {
    std::unordered_set<T> intersection;
    
    for (const auto& elem1 : set1) {
        if (set2.find(elem1) != set2.end()) {
            intersection.insert(elem1);
        }
    }
    
    return intersection;
}

#endif // STD_UNORDERED_SET_UTIL_H