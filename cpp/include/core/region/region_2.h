

#ifndef REGION_2_H
#define REGION_2_H

// include std libraries
#include <string>
#include <vector>
#include <memory>

// include CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polygon_2<K> Polygon_2;

class Region_2 : public Polygon_2 {
    private:
        double weight_visibility;  // 可視性の重み
        double weight_passability; // 移動の重み

    public:
        //** Constructor **//
        Region_2();
        Region_2(const Polygon_2 polygon_2);
        Region_2(const std::vector<Point_2>& region_points);
        Region_2(const Polygon_2 polygon_2, 
                 const double weight_visibility, 
                 const double weight_passability);
        Region_2(const std::vector<Point_2>& region_points, 
                 const double weight_visibility, 
                 const double weight_passability);
        
        //** Getter **//
        double get_weight_visibility() const;
        virtual double get_weight_passability() const;

        //** Setter **//
        void set_weight_visibility(const double weight_visibility);
        virtual void set_weight_passability(const double weight_passability);

        //** Construction Method **//
        /*************************************************
         * @brief 多角形を重み付き領域に変換する
         * 
         * @param polygons 
         * @return std::vector<std::shared_ptr<Region_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Region_2>> convert_polygons(const std::vector<Polygon_2>& polygons);
        /*************************************************
         * @brief 多角形を重み付き領域に変換する
         * 
         * @param polygons 
         * @param weights_visibility 
         * @param weights_passability 
         * @return std::vector<std::shared_ptr<Region_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Region_2>> convert_polygons(const std::vector<Polygon_2>& polygons, 
                                                                       const std::vector<double> weights_visibility, 
                                                                       const std::vector<double> weights_passability);
        
        //** Geometric Method **//
        /*************************************************
         * @brief 辺と交差するか判定する
         * 
         * @param segment_2 
         * @return true 
         * @return false 
         *************************************************/
        bool is_intersecting(const Segment_2 segment_2) const;

        //** File IO Method **//
        static std::vector<std::shared_ptr<Region_2>> read_regions(const std::string& abs_file_path);

}; 

#endif // REGION_2_H