

#ifndef EDGE_2_WTSN_H
#define EDGE_2_WTSN_H

// include header
#include "core/edge/edge_2.h"
#include "domain/wtsn/weight_passability_wtsn.h"


class Edge_2_WTSN : public Edge_2 {
    private:
        Weight_Passability_WTSN weight_passability_wtsn;
        static double activated_ratio; // 活性化判定のための係数
                                       // weight_passability_wtsn.damp_required_countに対する比率

    public:
        //** Constructor **//
        Edge_2_WTSN(std::shared_ptr<Node_2> source_ptr, 
                    std::shared_ptr<Node_2> target_ptr);

        //** Getter **//
        double get_weight_passability() const override;
        static double get_activated_ratio();

        //** Setter **//
        void set_weight_passability(const double weight) override;
        static void set_activated_ratio(const double activated_ratio);

        //** Simulation Functions **//
        void initialize();
        double damp();
        double amp(); 
        void fix();
        
        Weight_Passability_WTSN access_weight_passability_wtsn() const;
        void assign_weight_passability_wtsn();
        void assign_weight_passability_wtsn(const Weight_Passability_WTSN weight_passability_wtsn);
        bool is_activated();

};


#endif // EDGE_2_WTSN_H