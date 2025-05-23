
#ifndef REGION_2_WTSN_H
#define REGION_2_WTSN_H

// include std libraries
#include <string>
#include <vector>
#include <memory>

// include region_2
#include "core/region/region_2.h"

// include weight_passability_wtsn
#include "domain/wtsn/weight_passability_wtsn.h"


class Region_2_WTSN : public Region_2 {
    private:
        Weight_Passability_WTSN weight_passability_wtsn; // WTSN用重み

    public:
        //** Constructor **//
        Region_2_WTSN(const Polygon_2 polygon_2, 
                      const Weight_Passability_WTSN weight_passability_wtsn);
        
        //** Getter **//
        double get_weight_passability() const override;
        Weight_Passability_WTSN get_weight_passability_wtsn() const;

        //** Setter **//
        void set_weight_passability(const double weight_passability) override;
        void set_weight_passability_wtsn(const Weight_Passability_WTSN weight_passability_wtsn);

        //** Construction Method **//
        /*************************************************
         * @brief 多角形をWTSN用重み付き領域に変換する
         * 
         * @param polygons 
         * @param weights_passability_wtsn 
         * @return std::vector<std::shared_ptr<Region_2>> 
         *************************************************/
        static std::vector<std::shared_ptr<Region_2_WTSN>> convert_polygons(const std::vector<Polygon_2>& polygons, 
                                                                            const std::vector<Weight_Passability_WTSN> weights_passability_wtsn);
        
        //** Geometric Method **//

        //** File IO Method **//
        static std::vector<std::shared_ptr<Region_2_WTSN>> read_regions(const std::string& abs_file_path);

}; 

#endif // REGION_2_WTSN_H