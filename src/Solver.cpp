#include "Solver.h"
#include <cstdlib>

Solver::Solver(const Eigen::MatrixXd& P_mat, 
               const Eigen::VectorXd& q_vec, 
               const Eigen::MatrixXd& A_mat, 
               const Eigen::VectorXd& l_vec, 
               const Eigen::VectorXd& u_vec, 
               OSQPInt n, 
               OSQPInt m) 
{
    P = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
    A = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
    settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));

    // Populate P and A matrices
    set_data(P, P_mat);
    set_data(A, A_mat);

    // Set default settings
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter

    // Setup solver
    OSQPInt exitflag = osqp_setup(&solver, P, q_vec.data(), A, l_vec.data(), u_vec.data(), m, n, settings);
    if (exitflag != 0) {
        // Handle error
        throw std::runtime_error("OSQP setup failed.");
    }
}

Solver::~Solver() {
    osqp_cleanup(solver);
    free(A);
    free(P);
    free(settings);
}

void Solver::set_data(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat) {
   OSQPInt nnz = eigenMat.nonZeros();
    mat->n = eigenMat.cols();
    mat->m = eigenMat.rows();
    mat->p = (OSQPInt*)malloc((mat->n + 1) * sizeof(OSQPInt));
    mat->i = (OSQPInt*)malloc(nnz * sizeof(OSQPInt));
    mat->x = (OSQPFloat*)malloc(nnz * sizeof(OSQPFloat));

    // Eigen::MatrixXd를 Eigen::SparseMatrix로 변환
    Eigen::SparseMatrix<double> sparseMat = eigenMat.sparseView();

    // Fill in values
    OSQPInt current_index = 0;
    for (OSQPInt j = 0; j < mat->n; ++j) {
        mat->p[j] = current_index; // 현재 열의 시작 인덱스
        for (Eigen::SparseMatrix<double>::InnerIterator it(sparseMat, j); it; ++it) {
            mat->i[current_index] = it.row();
            mat->x[current_index] = static_cast<OSQPFloat>(it.value());
            current_index++;
        }
    }
    mat->p[mat->n] = current_index; // 마지막 열의 종료 인덱스
}

OSQPInt Solver::solve() {
    return osqp_solve(solver);
}
