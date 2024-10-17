#ifndef TEST_H
#define TEST_H


#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <osqp.h>

// CSCMatrix 클래스 정의
class CSCMatrix {
public:
    // 생성자: Eigen 행렬을 받아 CSC 형식으로 변환
    CSCMatrix(const Eigen::MatrixXd& matrix);

    // 비영 값 반환
    const std::vector<float>& get_values() const;
    // 행 인덱스 반환
    const std::vector<int>& get_row_indices() const;
    // 열 포인터 반환
    const std::vector<int>& get_col_pointers() const;
    // 비영 값 개수 반환
    int get_non_zero_count() const;
    // CSC 형식의 정보를 출력하는 함수
    void printCSC() const;

private:
    std::vector<float> values;     // 비영 값들#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <osqp.h>


class Solver {
    public:
        Solver(int n_x, int n_u, int NP);
        ~Solver();

        std::vector<double> solve(const Eigen::MatrixXd& P, const Eigen::VectorXd& q,
                              const Eigen::MatrixXd& constraint_A, const Eigen::VectorXd& UL,
                              const Eigen::VectorXd& LL);

    private:

        int n_x;
        int n_u;
        int NP;

        OSQPSolver* solver;
        std::unique_ptr<OSQPSettings> settings;

        // OSQPCscMatrix* P;
        // OSQPCscMatrix* A;

        std::vector<double> q;
    
        // CSC 형식으로 저장할 멤버 변수
        std::vector<double> P_data;   // P 행렬의 비영 값
        std::vector<int> P_indices;    // P 행렬의 행 인덱스
        std::vector<int> P_indptr;     // P 행렬의 열 포인터

        std::vector<double> A_data;   // A 행렬의 비영 값
        std::vector<int> A_indices;    // A 행렬의 행 인덱스
        std::vector<int> A_indptr;     // A 행렬의 열 포인터

        OSQPInt P_nnz;  // P의 비영 값 개수
        OSQPInt A_nnz;  // A의 비영 값 개수

        OSQPInt n;
        OSQPInt m;

        std::unique_ptr<OSQPCscMatrix> P_array;
        std::unique_ptr<OSQPCscMatrix> A_array;
        std::unique_ptr<OSQPFloat[]> P_data_array;
        std::unique_ptr<OSQPInt[]> P_indices_array;
        std::unique_ptr<OSQPInt[]> P_indptr_array;
        std::unique_ptr<OSQPFloat[]> A_data_array;
        std::unique_ptr<OSQPInt[]> A_indices_array;
        std::unique_ptr<OSQPInt[]> A_indptr_array;
        std::unique_ptr<OSQPFloat[]>  q_array;
        std::unique_ptr<OSQPFloat[]>  UL_array;
        std::unique_ptr<OSQPFloat[]>  LL_array;

        void cleanup();
        void print_data_vector(const std::vector<double>& data_vector);
        void setup_solver();

        void dense_to_CSC(const Eigen::MatrixXd& matrix, std::vector<double>& data,
                    std::vector<int>& indices, std::vector<int>& indptr, OSQPInt& nnz);

        void convert_to_OSQP_format(const std::vector<double>& data, 
                          const std::vector<int>& indices, 
                          const std::vector<int>& indptr,
                          std::unique_ptr<OSQPFloat[]>& data_array,
                          std::unique_ptr<OSQPInt[]>& indices_array,
                          std::unique_ptr<OSQPInt[]>& indptr_array);

        void convert_vector_to_OSQP_format(const Eigen::VectorXd& input_vector, 
                                std::unique_ptr<OSQPFloat[]>& output_array);


};

#endif // SOLVER_H

    std::vector<int> row_indices;  // 비영 값이 위치한 행 인덱스
    std::vector<int> col_pointers; // 각 열에서 비영 값이 시작하는 위치
    int non_zero_count;            // 비영 값의 개수
};

// Solver 클래스 정의
class Solver {
private:
    OSQPWorkspace* solver;
    std::unique_ptr<OSQPSettings> settings;
    std::unique_ptr<OSQPCscMatrix> P;
    std::unique_ptr<OSQPCscMatrix> A;

public:
    Solver();
    ~Solver();

    // 문제를 해결하는 메서드
    Eigen::VectorXd solve(const Eigen::MatrixXd& input_data);

    // CSC 형식으로 변환하는 메서드
    void convertToCSC(const Eigen::MatrixXd& input_data, CSCMatrix& csc_matrix);

private:
    void setup_problem_data();
    void setup_solver();
    void cleanup();
    void print_data_vector(const std::vector<double>& data_vector);
};




#endif