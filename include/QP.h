#ifndef QP_H
#define QP_H

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "DatabaseConfig.h"
#include "osqp.h"


class QP {
   public:
    QP(std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~QP();

    void set_data(const Eigen::MatrixXd& P_mat, 
                  const Eigen::MatrixXd& q_mat, 
                  const Eigen::MatrixXd& A_mat, 
                  const Eigen::VectorXd& l_vec, 
                  const Eigen::VectorXd& u_vec); 

    OSQPInt solve();
    std::vector<double> get_raw_result() const;


   private:
    void eigenmatrix2osqpmatrix(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat);
    
    std::shared_ptr<DatabaseConfig>& databaseconfig;
    
    int NP;
    int n_u;

    OSQPInt n, m;
    OSQPSolver* solver;
    OSQPCscMatrix* P;
    OSQPCscMatrix* A;
    OSQPSettings* settings;
    std::vector<double> raw_result;
    


};

#endif  // QP_H