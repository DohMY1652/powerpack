#include "Solver.h"
#include <stdexcept>

Solver::Solver(const Eigen::MatrixXd& P_mat, 
               const Eigen::VectorXd& q_vec, 
               const Eigen::MatrixXd& A_mat, 
               const Eigen::VectorXd& l_vec, 
               const Eigen::VectorXd& u_vec, 
               OSQPInt n, 
               OSQPInt m) 
    : n(n), m(m), result(n)
{
    // Use new instead of malloc
    P = new OSQPCscMatrix();
    A = new OSQPCscMatrix();
    settings = new OSQPSettings();
    solver = nullptr; // 초기화

    // Populate P and A matrices
    set_data(P, P_mat);
    set_data(A, A_mat);

    // Set default settings
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter
    settings->verbose = 0;  // 로그 출력을 비활성화

    // Setup solver
    OSQPInt exitflag = osqp_setup(&solver, P, q_vec.data(), A, l_vec.data(), u_vec.data(), m, n, settings);
    if (exitflag != 0) {
        // Handle error
        throw std::runtime_error("OSQP setup failed.");
    }
}

Solver::~Solver() {
    osqp_cleanup(solver);
    delete A; // Use delete instead of free
    delete P;
    delete settings;
}

void Solver::set_data(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat) {
    OSQPInt nnz = eigenMat.nonZeros();
    mat->n = eigenMat.cols();
    mat->m = eigenMat.rows();
    mat->p = new OSQPInt[mat->n + 1]; // Use new instead of malloc
    mat->i = new OSQPInt[nnz]; // Use new instead of malloc
    mat->x = new OSQPFloat[nnz]; // Use new instead of malloc

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
    OSQPInt exitflag = osqp_solve(solver);
    if (exitflag == 0) {
        // 결과를 벡터에 저장
        for (OSQPInt i = 0; i < n; ++i) {
            result[i] = static_cast<double>(solver->solution->x[i]);
        }
    }
    return exitflag; // 상태 코드 반환
}

std::vector<double> Solver::get_result() const {
    return result; // 결과 반환
}
