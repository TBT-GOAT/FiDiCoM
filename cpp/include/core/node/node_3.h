/*************************************************
 * @file node_3.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief 3D node class
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

#ifndef NODE_3_H
#define NODE_3_H

// include standards
#include <iostream>
#include <vector>
#include <memory>

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;

class Node_3 : public Point_3 {
    private:
        bool is_dummy = false; // ダミーノードかどうかのフラグ

    public:
        //** Constructor **//
        Node_3(bool is_dummy = false);
        Node_3(const double x, const double y, const double z, bool is_dummy = false);
        Node_3(const Point_3 p, bool is_dummy = false);
        
        //** Getter **//
        bool get_is_dummy() const { return is_dummy; }

        //** Geometric Method **//
};

#endif // NODE_3_H