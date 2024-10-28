//
// Created by joelo on 21.10.2024.
//
#include <Eigen/Dense>
#ifndef AIS4104_ASSIGNMENTS_SENSORFUSION_HPP
#define AIS4104_ASSIGNMENTS_SENSORFUSION_HPP


class SensorFusion {
public:
    SensorFusion(double mass, const Eigen::Vector3d& massCenter,
                 const Eigen::VectorXd& ftsBias, const Eigen::Vector3d& imuBias){

        Q_ << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3),
                Eigen::MatrixXd::Zero(3, 3), mass * Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3, 3),
                Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), mass * massCenter.cwiseAbs() * Eigen::MatrixXd::Identity(3,3);

        Eigen::MatrixXd Hf;
        Hf << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3, 3),
                Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd Ha;
        Ha << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3);


        Eigen::DiagonalMatrix<double> sigma_f(, , );

        Eigen::MatrixXd Rf;
        Rf << 0.3090, 0.1110, 1.4084,

    }


    //oppdater sensordata, synchronize timestamps
    void updateSensorData(const Eigen::VectorXd& ftsData, const Eigen::Vector3d& imuData,
                          const Eigen::Matrix3d& orientation, double timestamp);

// function for running kalman filter

private:
    Eigen::VectorXd state_;
    Eigen::MatrixXd A_, Q_, R_, P_;
    Eigen::MatrixXd z_;


    Eigen::VectorXd ftsBias_;
    Eigen::Vector3d imuBias_;
    double mass_;
    Eigen::Vector3d massCenter_;

    void predict();
    void update(const Eigen::VectorXd& measurement);
};

#endif //AIS4104_ASSIGNMENTS_SENSORFUSION_HPP
