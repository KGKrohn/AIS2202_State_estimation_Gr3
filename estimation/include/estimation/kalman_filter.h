#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

namespace estimation {
    class kalman_filter {
    public:
        kalman_filter();

        kalman_filter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0): x_(x0), P_(P0) {
            state_variable_names = {"ax", "ay", "az", "fx", "fy", "fz", "tx", "ty", "tz"};
        };


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
        bool write_state_to_csv(const std::string& filename, bool append = true) const {
            std::ofstream file;
            //Åpning av fil
            if (append) {
                file.open(filename, std::ios::app);
            } else {
                file.open(filename);
                for (size_t i = 0; i < state_variable_names.size(); ++i) {
                    file << state_variable_names[i];
                    if (i < state_variable_names.size() - 1) {
                        file << ",";
                    }
                }
                file << "\n";
            }

            if (!file.is_open()) {
                return false;
            }
            //skriving av data
            for (int i = 0; i < x_.size(); ++i) {
                file << x_(i);
                if (i < x_.size() - 1) {
                    file << ",";
                }
            }
            file << "\n";

            file.close();
            return true;
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
        std::vector<std::string> state_variable_names;
    };
}

#endif

