/*************************************************
 * @file arithmetic_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief arithmetic utility functions
 * @version 1.0
 * @date 2025-04-29
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

#ifndef ARITHMETIC_UTIL_H
#define ARITHMETIC_UTIL_H
 
// include standards
#include <iostream>
#include <float.h>
#include <math.h>
 
 
 
/**************************************************************************//**
 * @brief 浮動小数を考慮した二値比較（doubleとdouble）
 * 
 * @param x 値1
 * @param y 値2
 * @return int 1: 値1 > 値2、0: 値1 == 値2、-1: 値1 < 値2
******************************************************************************/
int cmp_vals(const double& x, const double& y, double rel_tolerance=DBL_EPSILON, double abs_epsilon=DBL_EPSILON);
/**************************************************************************//**
 * @brief 浮動小数を考慮した二値比較（floatとzfloat）
 * 
 * @param x 値1
 * @param y 値2
 * @return int 1: 値1 > 値2、0: 値1 == 値2、-1: 値1 < 値2
******************************************************************************/
int cmp_vals(const float& x, const float& y, float rel_tolerance=FLT_EPSILON, float abs_epsilon=FLT_EPSILON);
/**************************************************************************//**
 * @brief 浮動小数を考慮した二値比較（doubleとfloat）
 * 
 * @param x 値1
 * @param y 値2
 * @return int 1: 値1 > 値2、0: 値1 == 値2、-1: 値1 < 値2
******************************************************************************/
int cmp_vals(const double& x, const float& y, float rel_tolerance=FLT_EPSILON, float abs_epsilon=FLT_EPSILON);
/**************************************************************************//**
 * @brief 浮動小数を考慮した二値比較（floatとdouble）
 * 
 * @param x 値1
 * @param y 値2
 * @return int 1: 値1 > 値2、0: 値1 == 値2、-1: 値1 < 値2
******************************************************************************/
int cmp_vals(const float& x, const double& y, float rel_tolerance=FLT_EPSILON, float abs_epsilon=FLT_EPSILON);


#endif // ARITHMETIC_UTIL_H