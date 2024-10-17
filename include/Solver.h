#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Sparse> 
#include <Eigen/Dense>
#include "osqp.h"

class Solver {
public:
    Solver(const Eigen::MatrixXd& P, 
           const Eigen::VectorXd& q, 
           const Eigen::MatrixXd& A, 
           const Eigen::VectorXd& l, 
           const Eigen::VectorXd& u, 
           OSQPInt n, 
           OSQPInt m);
    ~Solver();
    OSQPInt solve();

private:
    OSQPSolver* solver;
    OSQPSettings* settings;
    OSQPCscMatrix* P;
    OSQPCscMatrix* A;

    void set_data(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat);
};

#endif // SOLVER_H
