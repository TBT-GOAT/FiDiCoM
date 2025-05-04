/*************************************************
 * @file std_string_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief string utility functions
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

#ifndef STD_STRING_UTIL_H
#define STD_STRING_UTIL_H

#include <string>
#include <algorithm>


/*************************************************
 * @brief ファイル拡張子を取得する
 * 
 * @param filename 
 * @return std::string 
 *************************************************/
inline std::string get_file_extension(const std::string& filename) {
    size_t dot_pos = filename.find_last_of('.');
    if (dot_pos == std::string::npos) {
        return ""; // 拡張子がない場合
    }
    std::string extension = filename.substr(dot_pos + 1);
    // 拡張子を小文字に変換
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    return extension;
}


#endif  // STD_STRING_UTIL_H