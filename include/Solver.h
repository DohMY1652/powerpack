#ifndef SOLVER_H
#define SOLVER_H

#include <osqp.h>
#include <iostream>
#include <memory>
#include <vector>
#include <ros/ros.h> 
#include <Eigen/Dense>

class Solver {
public:
    Solver();
    ~Solver();
    void solve(const std::vector<double>& input_data);
    void print_data_vector(const std::vector<double>& data_vector);

private:
    OSQPSolver* solver;
    std::unique_ptr<OSQPSettings> settings;

    OSQPFloat P_x[3];
    OSQPInt P_nnz;
    OSQPInt P_i[3];
    OSQPInt P_p[3];

    OSQPFloat A_x[4];
    OSQPInt A_nnz;
    OSQPInt A_i[4];
    OSQPInt A_p[3];
    
    OSQPFloat l[3];
    OSQPFloat u[3];
    OSQPInt n = 2;
    OSQPInt m = 3;

    std::unique_ptr<OSQPCscMatrix> P;
    std::unique_ptr<OSQPCscMatrix> A;


    void setup_problem_data();
    void setup_solver();
    void print_capabilities(OSQPInt cap);
    void cleanup();
};

#endif // SOLVER_H
