/*************************************************
 * @file std_list_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief list utility functions
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

#ifndef STD_LIST_UTIL_H
#define STD_LIST_UTIL_H

// include standards
#include <iostream>
#include <list>
#include <algorithm>

/**************************************************************************//**
 * @brief 入力した要素が何番目にあるかを検索する
 * 
 * @tparam T 
 * @param list ベクター
 * @param item 検索する要素
 * @return int 要素番号
******************************************************************************/
template <typename T>
int find_index(const std::list<T>& list, const T& item) {
    // 同じ要素が出てくるまで走査する
    auto ret = std::find(list.begin(), list.end(), item);

    // 要素が見つかった場合
    if (ret != list.end()) {
        return ret - list.begin();
    }
    
    // 要素が見つからなかった場合
    return -1;
}

#endif // STD_LIST_UTIL_H