/*************************************************
 * @file random_engine.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief random engine
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

#ifndef RANDOM_ENGINE_H
#define RANDOM_ENGINE_H

#include <random>

class Random_Engine {
    private:
        inline static unsigned int seed = 42; // 初期値を指定

    public:
        // 乱数エンジンのインスタンスを取得
        static std::mt19937& get_engine() {
            static std::mt19937 engine(seed); // シードを固定
            return engine;
        }

        // シードを設定する
        static void set_seed(unsigned int newSeed) {
            seed = newSeed;
            get_engine().seed(seed); // 再シード
        }
};


#endif // RANDOM_ENGINE_H