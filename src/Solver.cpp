#include "Solver.h"

Solver::Solver() : solver(nullptr), settings(nullptr), P(nullptr), A(nullptr) {
    setup_problem_data();
    setup_solver();
}

Solver::~Solver() {
    cleanup();
}

void Solver::solve(const std::vector<double>& input_data) {
    // 입력 데이터를 기반으로 문제를 해결
    // 여기에서 input_data를 사용하여 문제를 설정할 수 있습니다.

    // 예를 들어, input_data의 첫 번째 원소를 사용하여 q 값을 설정하는 경우
    OSQPFloat q[2] = { input_data[0], input_data[1] }; // 가정: input_data의 첫 2개 요소를 사용

    // 문제를 설정하고 해결
    OSQPInt exitflag = osqp_solve(solver);
    if (exitflag != 0) {
        std::cerr << "Error solving the problem: " << exitflag << std::endl;
    } else {
        ROS_INFO("Problem solved successfully.");
    }

    // 결과 출력 (여기서 더 복잡한 결과를 계산하여 출력할 수 있음)
    print_data_vector(input_data); // 입력 데이터를 출력
}

void Solver::setup_problem_data() {
    // Problem data
    OSQPFloat P_x[3] = { 4.0, 1.0, 2.0 };
    OSQPInt P_nnz = 3;
    OSQPInt P_i[3] = { 0, 0, 1 };
    OSQPInt P_p[3] = { 0, 1, 3 };

    OSQPFloat A_x[4] = { 1.0, 1.0, 1.0, 1.0 };
    OSQPInt A_nnz = 4;
    OSQPInt A_i[4] = { 0, 1, 0, 2 };
    OSQPInt A_p[3] = { 0, 2, 4 };
    
    OSQPFloat l[3] = { 1.0, 0.0, 0.0 };
    OSQPFloat u[3] = { 1.0, 0.7, 0.7 };
    OSQPInt n = 2;
    OSQPInt m = 3;

    // Allocate matrices using unique_ptr
    P = std::unique_ptr<OSQPCscMatrix>(new OSQPCscMatrix());
    A = std::unique_ptr<OSQPCscMatrix>(new OSQPCscMatrix());

    // Populate matrices
    csc_set_data(P.get(), n, n, P_nnz, P_x, P_i, P_p);
    csc_set_data(A.get(), m, n, A_nnz, A_x, A_i, A_p);
}

void Solver::setup_solver() {
    settings = std::unique_ptr<OSQPSettings>(new OSQPSettings());
    if (settings) {
        osqp_set_default_settings(settings.get());
        settings->polishing = 1;
    }

    OSQPInt cap = osqp_capabilities();
    print_capabilities(cap);

    // Setup solver
    OSQPInt exitflag = osqp_setup(&solver, P.get(), nullptr, A.get(), nullptr, nullptr, 3, 2, settings.get());
    if (exitflag != 0) {
        std::cerr << "Error setting up the solver: " << exitflag << std::endl;
    }
}

void Solver::print_capabilities(OSQPInt cap) {
    std::cout << "This OSQP library supports:\n";
    if (cap & OSQP_CAPABILITY_DIRECT_SOLVER) {
        std::cout << "    A direct linear algebra solver\n";
    }
    if (cap & OSQP_CAPABILITY_INDIRECT_SOLVER) {
        std::cout << "    An indirect linear algebra solver\n";
    }
    if (cap & OSQP_CAPABILITY_CODEGEN) {
        std::cout << "    Code generation\n";
    }
    if (cap & OSQP_CAPABILITY_DERIVATIVES) {
        std::cout << "    Derivatives calculation\n";
    }
    std::cout << "\n";
}

void Solver::cleanup() {
    if (solver) {
        osqp_cleanup(solver);
        solver = nullptr; // Prevent double cleanup
    }
}

void Solver::print_data_vector(const std::vector<double>& data_vector) {
    // 데이터 벡터의 내용을 출력
    ROS_INFO("Data Vector Contents:");
    for (size_t i = 0; i < data_vector.size(); ++i) {
        ROS_INFO("Element %zu: %f", i, data_vector[i]); // 각 원소를 출력
    }
}
