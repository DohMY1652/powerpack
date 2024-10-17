#include "Solver.h"

Solver::Solver(int n_x, int n_u, int NP) : 
    solver(nullptr), settings(nullptr), P_array(nullptr), A_array(nullptr), P_nnz(0), A_nnz(0),
    n_x(n_x), n_u(n_u), NP(NP), n(n_u*NP), m(n_u*NP) {

}

Solver::~Solver() {
    cleanup();
}

void Solver::cleanup() {
    if (solver) {
        osqp_cleanup(solver);
        solver = nullptr; // Prevent double cleanup
    }
}

void Solver::setup_solver() {
    settings = std::unique_ptr<OSQPSettings>(new OSQPSettings());
    if (settings) {
        osqp_set_default_settings(settings.get());
        settings->polishing = 1;
    }

    OSQPInt exitflag = osqp_setup(&solver, P_array.get(), nullptr, A_array.get(), nullptr, nullptr, 3, 2, settings.get());
    if (exitflag != 0) {
        std::cerr << "Error setting up the solver: " << exitflag << std::endl;
    }
}

void Solver::dense_to_CSC(const Eigen::MatrixXd& matrix, std::vector<double>& data,
                         std::vector<int>& indices, std::vector<int>& indptr, OSQPInt& nnz) {
    int rows = matrix.rows();
    int cols = matrix.cols();
    nnz = 0;

    // 열 포인터 초기화
    indptr.push_back(0);

    for (int col = 0; col < cols; ++col) {
        int nnz_in_col = 0;

        for (int row = 0; row < rows; ++row) {
            double value = matrix(row, col);
            if (value != 0.0) {
                data.push_back(value);
                indices.push_back(row);
                nnz_in_col++;
            }
        }

        nnz += nnz_in_col;
        indptr.push_back(indptr.back() + nnz_in_col);
    }
}

void Solver::convert_to_OSQP_format(const std::vector<double>& data, 
                          const std::vector<int>& indices, 
                          const std::vector<int>& indptr,
                          std::unique_ptr<OSQPFloat[]>& data_array,
                          std::unique_ptr<OSQPInt[]>& indices_array,
                          std::unique_ptr<OSQPInt[]>& indptr_array) {

    data_array = std::make_unique<OSQPFloat[]>(data.size());
    indices_array = std::make_unique<OSQPInt[]>(indices.size());
    indptr_array = std::make_unique<OSQPInt[]>(indptr.size());

    // 데이터 변환
    for (size_t i = 0; i < data.size(); ++i) {
        data_array[i] = static_cast<OSQPFloat>(data[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
        indices_array[i] = static_cast<OSQPInt>(indices[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
        indptr_array[i] = static_cast<OSQPInt>(indptr[i]);
    }                    
}

void Solver::convert_vector_to_OSQP_format(const Eigen::VectorXd& input_vector, 
                                std::unique_ptr<OSQPFloat[]>& output_array) {

    output_array = std::make_unique<OSQPFloat[]>(input_vector.size());
    
    for (size_t i = 0; i < input_vector.size(); ++i) {
        output_array[i] = static_cast<OSQPFloat>(input_vector(i));
    }
}

std::vector<double> Solver::solve(const Eigen::MatrixXd& P, const Eigen::VectorXd& q,
                                  const Eigen::MatrixXd& constraint_A, const Eigen::VectorXd& UL,
                                  const Eigen::VectorXd& LL) {
      

     // 상삼각 행렬로 변환
    Eigen::MatrixXd P_upperTriangular = P; 
    int rows = P_upperTriangular.rows();
    int cols = P_upperTriangular.cols();

    for (int i = 0; i < std::min(rows, cols); ++i) {
        for (int j = i + 1; j < rows; ++j) {
            if (P_upperTriangular(j, i) != 0) {
                double factor = P_upperTriangular(j, i) / P_upperTriangular(i, i);
                P_upperTriangular.row(j) -= factor * P_upperTriangular.row(i);
            }
        }
    }

    // P와 A를 CSC 형식으로 변환
    std::cout << "a" << std::endl;
    dense_to_CSC(P_upperTriangular, P_data, P_indices, P_indptr, P_nnz);
    dense_to_CSC(constraint_A, A_data, A_indices, A_indptr, A_nnz);
    std::cout << "b" << std::endl;

    // P와 A를 OSQP에 맞게 변환
    convert_to_OSQP_format(P_data, P_indices, P_indptr, P_data_array, P_indices_array, P_indptr_array);
    convert_to_OSQP_format(A_data, A_indices, A_indptr, A_data_array, A_indices_array, A_indptr_array);
    std::cout << "c" << std::endl;

    P_array = std::make_unique<OSQPCscMatrix>();
    A_array = std::make_unique<OSQPCscMatrix>();
    std::cout << "d" << std::endl;

    // P = new OSQPCscMatrix();
    // A = new OSQPCscMatrix();    

    csc_set_data(P_array.get(), n, n, P_nnz, P_data_array.get(), P_indices_array.get(), P_indptr_array.get());
    csc_set_data(A_array.get(), m, n, A_nnz, A_data_array.get(), A_indices_array.get(), A_indices_array.get());
    std::cout << "e" << std::endl;

    convert_vector_to_OSQP_format(q, q_array);
    convert_vector_to_OSQP_format(LL, LL_array);
    convert_vector_to_OSQP_format(UL, UL_array);

    settings = std::unique_ptr<OSQPSettings>(new OSQPSettings());
    std::cout << "g" << std::endl;

    if (settings) {
        osqp_set_default_settings(settings.get());
        settings->polishing = 1;
    }
    std::cout << "h" << std::endl;

    OSQPInt exitflag = osqp_setup(&solver, P_array.get(),q_array.get(), A_array.get(), LL_array.get(), UL_array.get(), m, n, settings.get());
    std::cout << "i" << std::endl;
    
    if (exitflag != 0) {
        std::cerr << "Error setting up the solver: " << exitflag << std::endl;
    }
    std::cout << "j" << std::endl;

    if (!exitflag) exitflag = osqp_solve(solver);
    std::cout << "k" << std::endl;

    std::vector<double> results(solver->solution->x, solver->solution->x + n_u);
    std::cout << "l" << std::endl;

    return results;
}
