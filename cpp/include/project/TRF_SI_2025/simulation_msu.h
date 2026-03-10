#ifndef SIMULATION_MSU_H
#define SIMULATION_MSU_H

// include standards
#include <vector>
#include <string>

class Parameters {
    public:
        double init_weight;         // 移動抵抗の初期値
        size_t damp_required_count; // 減衰にかかる回数
        size_t amp_required_count;  // 増幅にかかる回数
        double beta;                // 移動需要の距離減衰率
        size_t seed;                // ランダムエンジンのシード 

        //** Constructor **//
        Parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);
        Parameters(const std::vector<std::string>& args);

};


//** Simulation **//
Parameters parse_parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);
Parameters parse_parameters(const std::vector<std::string>& args);

int execute_simulation_msu(double init_w, size_t damp_count, size_t amp_count, double b, size_t s, const bool writing_nx);

int run_simulation_msu(const std::vector<std::string>& args, const bool writing_nx=false);
int run_simulation_msu(double init_w, size_t damp_count, size_t amp_count, double b, size_t s, const bool writing_nx=false);
int run_simulation_msu(const Parameters& params, const bool writing_nx=false);


#endif // SIMULATION_MSU_H