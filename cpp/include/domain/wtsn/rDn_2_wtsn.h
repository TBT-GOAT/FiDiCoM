
#ifndef RDN_2_WTSN_H
#define RDN_2_WTSN_H

// include rDn_2
#include "core/net/rDn_2.h"

// include region_2_wtsn
#include "domain/wtsn/region_2_wtsn.h"


class rDn_2_WTSN : public rDn_2 {
        
    public:
        
        //** Constructor **//
        using rDn_2::rDn_2;

        //** Network Method **//
        /*************************************************
         * @brief ランダムドロネー網を初期化する
         * 
         *************************************************/
        void initialize() override;
        /*************************************************
         * @brief 与えられた頂点でランダムドロネー網を初期化する
         * 
         * @param node_ptrs 
         *************************************************/
        void initialize(const std::vector<std::shared_ptr<Node_2>> node_ptrs) override;
        /*************************************************
         * @brief 重み付き領域と交差するエッジを重み付ける
         * 
         * @param region_ptrs 
         *************************************************/
        void weight_edges(std::vector<std::shared_ptr<Region_2_WTSN>> region_ptrs);
        /*************************************************
         * @brief エッジの重みをリセットする
         * 
         *************************************************/
        void reset_weight();
};

#endif // RDN_2_WTSN_H