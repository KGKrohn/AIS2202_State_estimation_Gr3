#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>

namespace estimation {
    class kalman_filter {
    public:
        kalman_filter();

        void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
            x_ = x0;
            P_ = P0;
        }

        void predict(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q,const Eigen::MatrixXd& B, const Eigen::VectorXd& u) {

            x_ = A * x_ + B*u;

            P_ = A * P_ * A.transpose() + Q;
        }

        void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {

            K_ = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

            x_ = x_ + K_ * (z - H * x_);

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
            P_ = (I - K_ * H) * P_;
        }

        Eigen::VectorXd get_state() const {
            return x_;
        }

        Eigen::MatrixXd get_covariance() const {
            return P_;
        }

    private:
        Eigen::VectorXd x_; // State estimate
        Eigen::MatrixXd P_; // State covariance
        Eigen::MatrixXd K_; // Kalman gain
    };
}

#endif
