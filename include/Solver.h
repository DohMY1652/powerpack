#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>
#include <vector>

class Solver {
public:
    Solver(const Eigen::MatrixXd& P_mat, 
           const Eigen::VectorXd& q_vec, 
           const Eigen::MatrixXd& A_mat, 
           const Eigen::VectorXd& l_vec, 
           const Eigen::VectorXd& u_vec, 
           OSQPInt n, 
           OSQPInt m);

    ~Solver();

    OSQPInt solve(); // 최적화 문제를 해결하는 메서드
    std::vector<double> get_result() const; // 결과를 반환하는 메서드

private:
    void set_data(OSQPCscMatrix* mat, const Eigen::MatrixXd& eigenMat); // 행렬 데이터를 설정하는 메서드

    OSQPCscMatrix* P; // 목적 함수 행렬
    OSQPCscMatrix* A; // 제약 조건 행렬
    OSQPSettings* settings; // OSQP 설정
    OSQPSolver* solver; // OSQP 솔버 작업공간
    std::vector<double> result; // 최적화 결과를 저장할 벡터
    OSQPInt n; // 변수의 수
    OSQPInt m; // 제약조건의 수
};

#endif // SOLVER_H
