#ifndef SIMULATION_MSU_H
#define SIMULATION_MSU_H

// include standards
#include <vector>
#include <string>

class Parameters {
    public:
        double init_weight;
        size_t damp_required_count;
        size_t amp_required_count;
        double beta;
        size_t seed;

        //** Constructor **//
        Parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);
        Parameters(const std::vector<std::string>& args);

};


//** Simulation **//
Parameters parse_parameters(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);
Parameters parse_parameters(const std::vector<std::string>& args);

int execute_simulation_msu(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);

int run_simulation_msu(const std::vector<std::string>& args);
int run_simulation_msu(double init_w, size_t damp_count, size_t amp_count, double b, size_t s);
int run_simulation_msu(const Parameters& params);


#endif // SIMULATION_MSU_H