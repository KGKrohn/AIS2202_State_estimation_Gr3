#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

namespace estimation {
    class kalman_filter {
    public:
        kalman_filter(const Eigen::MatrixXd& x0,
                      const Eigen::MatrixXd& P0,float m,
                      const Eigen::MatrixXd& r):
                      x_(x0), P_(P0), m_(m),r_(r),
                      Q(9,9),B(9,3),Rws_(3,3),g_(3,1),U(3,1) {
            state_variable_names = {"ax", "ay", "az", "fx", "fy", "fz", "tx", "ty", "tz"};
        };

        void update_static_variables(std::vector<float> SaSDa,
                                     std::vector<float> SfSDf,
                                     std::vector<float> StSDt,
                                     float SDK) {
            SaSDa_ = SaSDa;
            SfSDf_ = SfSDf;
            StSDt_ = StSDt;
            Q_calc(SDK);
            B_calc();
            U<< 0,0,0;
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

        void Q_calc(float SDk) {
            Eigen::MatrixXd Q_(9,9);
            Q_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, m_, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, m_, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, m_, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, m_*abs(r_(0,0)), 0, 0,
                  0, 0, 0, 0, 0, 0, 0, m_*abs(r_(1,0)), 0,
                  0, 0, 0, 0, 0, 0, 0, 0, m_*abs(r_(2,0));
            Q = Q_* SDk;
        }

        Eigen::MatrixXd rs_sqew_matrix() {
            Eigen::MatrixXd rs_sqew(3,3);
            rs_sqew << 0,                 -r_(2,0) , r_(1,0),
                       r_(2,0),  0                 , -r_(0,0),
                       -r_(1,0), r_(0,0)  , 0;
            return rs_sqew;
        }

        void B_calc() {
            Eigen::MatrixXd rs_sqew = rs_sqew_matrix();
            B << 1, 0, 0,
                 0, 1, 0,
                 0, 0, 1,
                 m_, 0, 0,
                 0, m_, 0,
                 0, 0, m_,
                 m_*rs_sqew(0,0), m_*rs_sqew(0,1), m_*rs_sqew(0,2),
                 m_*rs_sqew(1,0), m_*rs_sqew(1,1), m_*rs_sqew(1,2),
                 m_*rs_sqew(2,0), m_*rs_sqew(2,1), m_*rs_sqew(2,2);
        }

        Eigen::MatrixXd Hf_matrix() {
            Eigen::MatrixXd Hf(6,9);
            Hf <<   0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 1;
            return Hf;
        }

        Eigen::MatrixXd Rf_matrix() {
            Eigen::MatrixXd Rf(6,6);
            Rf <<   SfSDf_[0], 0, 0, 0, 0, 0,
                    0, SfSDf_[1], 0, 0, 0, 0,
                    0, 0, SfSDf_[2], 0, 0, 0,
                    0, 0, 0, StSDt_[0], 0, 0,
                    0, 0, 0, 0, StSDt_[1], 0,
                    0, 0, 0, 0, 0, StSDt_[2];
            return Rf;
        }

        Eigen::MatrixXd Ra_matrix() {
            Eigen::MatrixXd Ra(3,3);
            Ra <<   SaSDa_[0], 0, 0,
                    0, SaSDa_[1], 0,
                    0, 0, SaSDa_[2];
            return Ra;
        }

        Eigen::MatrixXd Ha_matrix() {
            Eigen::MatrixXd Ha(3,9);
            Ha <<   1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0;
            return Ha;
        }

        std::vector<double> scale_time_array(std::vector<double> time_array) {
            double start = time_array.front();
            for (int i = 0; i < time_array.size(); i++)
            {
                time_array[i] = time_array[i]- start;
            }
            return time_array;
        }

        void Rws(float r11,float r12,float r13,float r21,float r22,float r23,float r31,float r32,float r33) {
            Rws_ << r11, r12, r13,
                    r21, r22, r23,
                    r31, r32, r33;
        }

        void uk(float ax,float ay,float az, float fr, float ff, float fa) {
            Eigen::MatrixXd gw(3,1);
            gw << ax,ay,az;
            Eigen::MatrixXd gs  = Rws_.transpose()*gw;

            U = (gs - g_) * (fr / (ff - fa));
            g_ = gs;
        }

        Eigen::MatrixXd Zc_matrix(Eigen::MatrixXd Xest) {
            Eigen::MatrixXd rs_sqew = rs_sqew_matrix();
            Eigen::MatrixXd Z(6,9);
            Z << -m_, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, -m_, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, -m_, 0, 0, 1, 0, 0, 0,
                    -m_*rs_sqew(0,0), -m_*rs_sqew(0,1), -m_*rs_sqew(0,2), 0, 0, 0, 1, 0, 0,
                    -m_*rs_sqew(1,0), -m_*rs_sqew(1,1), -m_*rs_sqew(1,2), 0, 0, 0, 0, 1, 0,
                    -m_*rs_sqew(2,0), -m_*rs_sqew(2,1), -m_*rs_sqew(2,2), 0, 0, 0, 0, 0, 1;
            Eigen::MatrixXd Zc = Z*Xest;
            return Zc;
        }

        void  zf_update(float ax,float ay, float az, float fx, float fy, float fz, float tx, float ty, float tz) {
            Eigen::MatrixXd x(9,1);
            x << ax, ay, az, fx, fy, fz, tx, ty, tz;
            H_ = Hf_matrix();
            Z_ = H_*x;
            R_ = Rf_matrix();
        }

        void za_update(float ax,float ay, float az, float fx, float fy, float fz, float tx, float ty, float tz) {
            Eigen::MatrixXd x(9,1);
            x << ax, ay, az, fx, fy, fz, tx, ty, tz;
            H_ = Ha_matrix();
            Z_ = H_*x;
            R_ = Ra_matrix();
        }

        void predict() {
            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);

            x_ = A * x_ + B*U;
            P_ = A * P_ * A.transpose() + Q;
        }
        


        void update() {
            K_ = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
            x_ = x_ + K_ * (Z_ - H_ * x_);
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
            P_ = (I - K_ * H_) * P_;
        }

        Eigen::VectorXd get_state() const {
            return x_;
        }

        Eigen::MatrixXd get_covariance() const {
            return P_;
        }



    private:
        std::vector<std::string> state_variable_names;
        float m_;
        Eigen::MatrixXd x_; // State estimate
        Eigen::MatrixXd r_; // State estimate
        Eigen::MatrixXd P_; // State covariance
        Eigen::MatrixXd K_; // Kalman gain
        Eigen::MatrixXd Q; //
        Eigen::MatrixXd Z_; //
        Eigen::MatrixXd H_; //
        Eigen::MatrixXd R_; //
        Eigen::MatrixXd U; //
        Eigen::MatrixXd B; // B
        Eigen::MatrixXd Rws_;
        Eigen::MatrixXd g_;
        std::vector<float> SaSDa_;
        std::vector<float> SfSDf_;
        std::vector<float> StSDt_;
    };
}

#endif
