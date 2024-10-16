#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Powerpack.h"
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>



#include <osqp.h>
#include <memory>


void printDataVector(const std::vector<double>& data_vector) {
    // 데이터 벡터의 내용을 출력
    ROS_INFO("Data Vector Contents:");
    for (size_t i = 0; i < data_vector.size(); ++i) {
        ROS_INFO("Element %zu: %f", i, data_vector[i]); // 각 원소를 출력
    }
}

#include <iostream>
#include <Eigen/Dense>
#include <vector>

// CSC 형식을 나타내는 클래스 정의
class CSCMatrix {
private:
    std::vector<float> values;     // 비영 값들
    std::vector<int> row_indices;  // 비영 값이 위치한 행 인덱스
    std::vector<int> col_pointers; // 각 열에서 비영 값이 시작하는 위치
    int non_zero_count;            // 비영 값의 개수

public:
    // 생성자: Eigen 행렬을 받아 CSC 형식으로 변환
    CSCMatrix(const Eigen::MatrixXf& matrix) {
        int rows = matrix.rows();
        int cols = matrix.cols();
        non_zero_count = 0; // 초기화

        // 열 포인터는 각 열의 시작 인덱스를 가리키며, 첫 번째 값은 항상 0입니다.
        col_pointers.push_back(0);

        // 열 단위로 순회하며 CSC 데이터를 구성합니다.
        for (int col = 0; col < cols; ++col) {
            int nnz_in_column = 0; // 현재 열의 비영 값 수를 계산하기 위한 변수
            for (int row = 0; row < rows; ++row) {
                float value = matrix(row, col);
                if (value != 0.0f) {
                    // 비영 값일 경우, 값을 저장하고 해당 행의 인덱스를 기록합니다.
                    values.push_back(value);
                    row_indices.push_back(row);
                    ++nnz_in_column;
                }
            }
            // 비영 값 개수 갱신
            non_zero_count += nnz_in_column;

            // 각 열의 시작 위치를 기록합니다.
            col_pointers.push_back(col_pointers.back() + nnz_in_column);
        }
    }

    // 비영 값 반환
    const std::vector<float>& getValues() const {
        return values;
    }

    // 행 인덱스 반환
    const std::vector<int>& getRowIndices() const {
        return row_indices;
    }

    // 열 포인터 반환
    const std::vector<int>& getColPointers() const {
        return col_pointers;
    }

    // 비영 값 개수 반환
    int getNonZeroCount() const {
        return non_zero_count;
    }

    // CSC 형식의 정보를 출력하는 함수
    void printCSC() const {
        std::cout << "CSC 형식 - 비영 값 (values): ";
        for (float value : values) {
            std::cout << value << " ";
        }
        std::cout << std::endl;

        std::cout << "CSC 형식 - 행 인덱스 (row_indices): ";
        for (int row_idx : row_indices) {
            std::cout << row_idx << " ";
        }
        std::cout << std::endl;

        std::cout << "CSC 형식 - 열 포인터 (col_pointers): ";
        for (int col_ptr : col_pointers) {
            std::cout << col_ptr << " ";
        }
        std::cout << std::endl;

        std::cout << "비영 값 개수: " << non_zero_count << std::endl;
    }
};




int main(int argc, char* argv[]) {

      // 예시 행렬 (Eigen::MatrixXf)
    Eigen::MatrixXf matrix(4, 4);
    matrix << 1.0, 0.0, 3.0, 0.0,
              0.0, 4.0, 0.0, 0.0,
              5.0, 0.0, 6.0, 0.0,
              0.0, 0.0, 0.0, 7.0;

    // 행렬을 CSC 형식으로 변환하는 클래스 생성
    CSCMatrix cscMatrix(matrix);

    // CSC 형식의 정보를 출력
    cscMatrix.printCSC();


  ////////////////yaml 파일 터미널에서 직접 받기 /////////////////
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_yaml_file>" << std::endl;
        return 1;
    }
    std::string filename = argv[1];
    YAML::Node config = YAML::LoadFile(filename);
    DatabaseConfig data(config);
//////////////////////////////////////////////////////

    ros::init(argc, argv, "Pos_neg_controller");
    ros::NodeHandle n;

///////////////parameter server에서 yaml 데이터 받기//////////////////
    // std::string yaml_file;
    // if (!n.getParam("yaml_file", yaml_file)) {
    //     ROS_ERROR("Could not find parameter 'yaml_file'");
    //     return 1;
    // }

    // YAML::Node config = YAML::LoadFile(yaml_file);
    // DatabaseConfig data(config);

 ///////////////////////////////////////////////////////////////////   
 
    Powerpack powerpack(data.get_n_pos_channel(), data.get_n_neg_Channel(), data.get_MPC_parameters());

    ros::Subscriber sen_sub = n.subscribe<std_msgs::Float32MultiArray>("sen_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        for (const auto& value : data->data) {
            data_vector.push_back(static_cast<double>(value));
        }
        powerpack.update_sensor(data_vector);
        std::cout << "sensor data" << std::endl;
        printDataVector(powerpack.get_all_sensor_data());
        
      } 
    );

    ros::Subscriber ref_sub = n.subscribe<std_msgs::Float32MultiArray>("ref_values", 100, [&powerpack](const boost::shared_ptr<const std_msgs::Float32MultiArray>& data) {
        std::vector<double> data_vector;
        data_vector.reserve(data->data.size());

        for (const auto& value : data->data) {
            data_vector.push_back(static_cast<double>(value));
        }
        powerpack.update_reference(data_vector);
         std::cout << "sensor data" << std::endl;
        printDataVector(powerpack.get_all_reference_data());
      } 
    );

    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt16MultiArray>("mpc_pwm", 10);
    ros::Rate loop_rate(100); // 100 Hz

    while (ros::ok())
  {


    powerpack.run();

    std::vector<unsigned int> data = powerpack.get_control_signal();
    std_msgs::UInt16MultiArray msg;
    msg.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
    msg.data[i] = data[i];
    }
    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


