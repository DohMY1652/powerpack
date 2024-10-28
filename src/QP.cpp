#include "QP.h"

#include <iostream>
#include <vector>

#include "DatabaseConfig.h"

QP::QP(std::shared_ptr<DatabaseConfig>& databaseconfig)
    : databaseconfig(databaseconfig),
    solver(nullptr) {
         std::vector<double> parameters = databaseconfig->get_MPC_parameters();
        NP = (int)parameters[0];
        n_u = (int)parameters[2];
        n = n_u*NP;
        m = n_u*NP;
        P = new OSQPCscMatrix();
        A = new OSQPCscMatrix();
        settings = new OSQPSettings();
        osqp_set_default_settings(settings);
        settings->alpha = 1.0;
        settings->verbose = 0;
    }

QP::~QP() {
    osqp_cleanup(solver);
    delete A;
    delete P;
    delete settings;
}





void QP::set_data(const Eigen::MatrixXd& P_mat, 
                  const Eigen::MatrixXd& q_mat,
                  const Eigen::MatrixXd& A_mat, 
                  const Eigen::VectorXd& l_vec, 
                  const Eigen::VectorXd& u_vec) 
{
    
    eigenmatrix2osqpmatrix(P, P_mat);
    eigenmatrix2osqpmatrix(A, A_mat);

    if (solver) {
        osqp_cleanup(solver);
        solver = nullptr;
    }

    Eigen::VectorXd q_vec = q_mat.col(0);

    OSQPInt exitflag = osqp_setup(&solver, P, q_vec.data(), A, l_vec.data(), u_vec.data(), m, n, settings);

    
    if (exitflag != 0) {
        throw std::runtime_error("OSQP setup failed.");
    }
}


void QP::eigenmatrix2osqpmatrix(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat) {
    OSQPInt nnz = eigenMat.nonZeros();
    mat->n = eigenMat.cols();
    mat->m = eigenMat.rows();
    mat->p = new OSQPInt[mat->n + 1];
    mat->i = new OSQPInt[nnz];
    mat->x = new OSQPFloat[nnz];

    // Eigen::MatrixXd를 Eigen::SparseMatrix로 변환
    Eigen::SparseMatrix<double> sparseMat = eigenMat.sparseView();

    // Fill in values
    OSQPInt current_index = 0;
    for (OSQPInt j = 0; j < mat->n; ++j) {
        mat->p[j] = current_index;
        for (Eigen::SparseMatrix<double>::InnerIterator it(sparseMat, j); it; ++it) {
            mat->i[current_index] = it.row();
            mat->x[current_index] = static_cast<OSQPFloat>(it.value());
            current_index++;
        }
    }
    mat->p[mat->n] = current_index;
}

OSQPInt QP::solve() {
    OSQPInt exitflag = osqp_solve(solver);

    if (!solver || !solver->solution) {
        std::cerr << "Solver or solution is null!" << std::endl;
        return -1;
    }

    if (exitflag == 0) {
        if (n > 0) {
            raw_result.resize(n_u);
            for (OSQPInt i = 0; i < n_u; ++i) {
                raw_result[i] = static_cast<double>(solver->solution->x[i]);
            }
        }
    }
    return exitflag;
}

std::vector<double> QP::get_raw_result() const {
    return raw_result;
}