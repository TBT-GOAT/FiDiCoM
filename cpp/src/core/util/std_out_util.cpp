/**************************************************************************//**
 * @file std_out_util.cpp
 * @author tabatash (tabatash@kajima.com)
 * @brief 標準出力用の便利関数
 * @version 0.1
 * @date 2024-01-29
 * 
 * Copyright (c) 2024 TABATA Shota All rights reserved.
 * 
******************************************************************************/

// include header
#include "core/util/std_out_util.h"

void show_progress_bar(const int& numerator, const int& denominator, const int& width, const bool& show_percentage) {
    
    double percentage {static_cast<double>(numerator) / denominator};   // 進捗率
    int filled_width {static_cast<int>(percentage * width)};            // 進捗率に応じて埋めるバーの長さ
    int empty_width {width - filled_width};                             // 空白の長さ

    std::cout << "[";

    for (size_t i {0}; i < filled_width; i++) {
        std::cout << "=";
    }
    for (int i = 0; i < empty_width; i++) {
        std::cout << " ";
    }

    if (show_percentage) 
        std::cout << "] " << static_cast<int>(percentage * 100) << "%\r";  // パーセント表示
    else 
        std::cout << "] " << numerator << " / " << denominator << "\r";    // 分数表示 
    
    std::cout.flush(); // 進捗が完了したらプログレスバーを非表示にする
}

void pausing() {
    int flag;
    std::cout << "Press Enter to continue...";
    flag = std::cin.get();
}