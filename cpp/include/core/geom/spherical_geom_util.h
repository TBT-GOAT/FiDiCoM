/*************************************************
 * @file spherical_geom_util.h
 * @author Shota TABATA (tbtgoat.contact@gmail.com)
 * @brief spherical geometry utility functions
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


#ifndef SPHERICAL_GEOM_UTIL_H
#define SPHERICAL_GEOM_UTIL_H

// include CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Projection_on_sphere_traits_3.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;

typedef K::Point_2                                              Point_2;
typedef K::Vector_2                                             Vector_2;

typedef CGAL::Projection_on_sphere_traits_3<K>                  Traits;
typedef CGAL::Delaunay_triangulation_on_sphere_2<Traits>        DToS2;
typedef K::Point_3                                              Point_3;
typedef Traits::Point_3                                         Point_on_sphere_3;
typedef K::Vector_3                                             Vector_3;


/*************************************************
 * @brief ふたつのベクトルがなす角を求める
 * 
 * @param v1 
 * @param v2 
 * @return double 
 *************************************************/
double calc_angle(const Vector_2& v1, const Vector_2& v2);

/*************************************************
 * @brief 中心に対する角度を求める
 * 
 * @param center_point 
 * @param target_point 
 * @return double 
 *************************************************/
double calc_angle(const Point_2& center_point, const Point_2& target_point);

/*************************************************
 * @brief ふたつのベクトルがなす角を求める
 * 
 * @param v1 
 * @param v2 
 * @return double 
 *************************************************/
double calc_angle(const Vector_3& v1, const Vector_3& v2);

/*************************************************
 * @brief 球面上の内角を求める
 * 
 * @param center_point 
 * @param curr_point 
 * @param prev_point 
 * @param next_point 
 * @return double 
 *************************************************/
double calc_inner_angle_on_sphere(const Point_3& center_point, 
                                  const Point_on_sphere_3& curr_point, 
                                  const Point_on_sphere_3& prev_point, 
                                  const Point_on_sphere_3& next_point);

/*************************************************
 * @brief ある中心に対する2つの点がなす中心角を求める
 * 
 * @param center_point 
 * @param from_point 
 * @param to_point 
 * @return double 
 *************************************************/
double calc_center_angle (const Point_2& center_point, 
                          const Point_2& from_point, 
                          const Point_2& to_point);

/*************************************************
 * @brief 球面上の多角形の立体角を求める
 * 
 * @param center_point 
 * @param vertices 
 * @param radius 
 * @return double 
 *************************************************/
double calc_solid_angle(const Point_3& center_point, 
                        const std::vector<Point_on_sphere_3>& vertices, 
                        double radius = 1.0) ;

/*************************************************
 * @brief 円周上でのボロノイ図を得る
 * 
 * @param center_point 
 * @param generators 
 * @return std::unordered_map<Point_2, std::pair<Point_2, Point_2>> 
 *************************************************/
std::unordered_map<Point_2, std::pair<Point_2, Point_2>> construct_circular_Voronoi_diagram(const Point_2& center_point, 
                                                                                            const std::vector<Point_2>& generators, 
                                                                                            const double radius = 1.0);

/*************************************************
 * @brief 球面ボロノイ図を得る
 * 
 * @param center_point 
 * @param generators 
 * @return std::unordered_map<Point_3, std::vector<Point_on_sphere_3>> 
 *************************************************/
std::unordered_map<Point_3, std::vector<Point_on_sphere_3>> construct_spherical_Voronoi_diagram(const Point_3& center_point, 
                                                                                                const std::vector<Point_3>& generators, 
                                                                                                const double radius = 1.0);
                                                                                                

#endif //SPHERICAL_GEOM_UTIL_H