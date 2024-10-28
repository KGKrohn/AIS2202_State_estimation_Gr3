//
// Created by joelo on 21.10.2024.
//
#include "SensorFusion.hpp"


SensorFusion::SensorFusion(double mass, const Eigen::Vector3d &massCenter, const Eigen::VectorXd &ftsBias,
                           const Eigen::Vector3d &imuBias) {
    A_ = Eigen::MatrixXd::Identity(9, 9);
    z_ << Eigen::MatrixXd::Identity(3, 3) * -mass_, Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
    
}



void SensorFusion::updateSensorData(const Eigen::VectorXd &ftsData, const Eigen::Vector3d &imuData,
                                    const Eigen::Matrix3d &orientation, double timestamp) {

}