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
            state_variable_names = {"ax", "ay", "az", "fx", "fy", "fz", "tx", "ty", "tz","t"};
            z_variable_names = {"fx", "fy", "fz", "tx", "ty", "tz","t"};
        };

        void update_static_variables(std::vector<float> SaSDa,
                                     std::vector<float> SfSDf,
                                     std::vector<float> StSDt) {
            SaSDa_ = SaSDa;
            SfSDf_ = SfSDf;
            StSDt_ = StSDt;
            B_calc();
            U<< 0,0,0;
            prev_t = 0;
        }

        bool write_state_to_csv(const std::string& filename, bool append = true, float t=0.0){
            std::ofstream file;

            //Åpning av fil
            if (append) {
                file.open(filename, std::ios::out | std::ios::app);
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
            file << ",";
            file << t;
            file << "\n";

            file.close();
            return true;
        }

        bool write_z_to_csv(const std::string& filename, bool append = true, float t=0.0){
            std::ofstream file;

            //Åpning av fil
            if (append) {
                file.open(filename, std::ios::out | std::ios::app);
            } else {
                file.open(filename);
                for (size_t i = 0; i < z_variable_names.size(); ++i) {
                    file << z_variable_names[i];
                    if (i < z_variable_names.size() - 1) {
                        file << ",";
                    }
                }
                file << "\n";
            }
            if (!file.is_open()) {
                return false;
            }

            //skriving av data
            for (int i = 0; i < Z_.size(); ++i) {
                file << Z_(i);
                if (i < Z_.size() - 1) {
                    file << ",";
                }
            }
            file << ",";
            file << t;
            file << "\n";

            file.close();
            return true;
        }

        void Q_calc(float SDk, float t) {
            Eigen::MatrixXd Q_(9,9);
            Q_ << SDk, 0, 0, 0, 0, 0, 0, 0, 0,
                  0,SDk, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, SDk, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, m_*SDk, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, m_*SDk, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, m_*SDk, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, m_*abs(r_(0,0))*SDk, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, m_*abs(r_(1,0))*SDk, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, m_*abs(r_(2,0))*SDk;
            float d_t = (t-prev_t)*0.00001;
            Q = d_t*Q_;
            prev_t = t;
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

        Eigen::MatrixXd Ha_matrix() {
            Eigen::MatrixXd Ha(3,9);
            Ha <<   1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0;
            return Ha;
        }

        Eigen::MatrixXd Hc_matrix() {
            Eigen::MatrixXd Hc(6,9);
            Eigen::MatrixXd rs_sqew = rs_sqew_matrix();
            Hc <<   -m_*1, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, -m_*1, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, -m_*1, 0, 0, 1, 0, 0, 0,
                    rs_sqew(0,0)*-m_, rs_sqew(0,1)*-m_, rs_sqew(0,2)*-m_, 0, 0, 0, 1, 0, 0,
                    rs_sqew(1,0)*-m_, rs_sqew(1,1)*-m_, rs_sqew(1,2)*-m_, 0, 0, 0, 0, 1, 0,
                    rs_sqew(2,0)*-m_, rs_sqew(2,1)*-m_, rs_sqew(2,2)*-m_, 0, 0, 0, 0, 0, 1;
            return Hc;
        }
        Eigen::MatrixXd Hc_matrix2() {
            Eigen::MatrixXd Hc(6,9);
            Hc <<   1, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 1, 0, 0, 0,
                    1, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 1;
            return Hc;
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

        std::vector<double> scale_time_array(std::vector<double> time_array, float phase_shift = 0.0) {
            double start = time_array.front();
            for (int i = 0; i < time_array.size(); i++)
            {
                time_array[i] = time_array[i]- start + phase_shift;
            }
            return time_array;
        }

        void Rws(float r11,float r12,float r13,float r21,float r22,float r23,float r31,float r32,float r33) {
            Rws_ << r11, r12, r13,
                    r21, r22, r23,
                    r31, r32, r33;
        }

        void zc_update(float ax,float ay, float az, float fx, float fy, float fz, float tx, float ty, float tz) {
            Eigen::MatrixXd x(9,1);
            x << ax, ay, az, fx, fy, fz, tx, ty, tz;
            Eigen::MatrixXd rs_sqew = rs_sqew_matrix();
            Eigen::MatrixXd Z(6,9);
            Z <<  -m_, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, -m_, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, -m_, 0, 0, 1, 0, 0, 0,
                    -m_*rs_sqew(0,0), -m_*rs_sqew(0,1), -m_*rs_sqew(0,2), 0, 0, 0, 1, 0, 0,
                    -m_*rs_sqew(1,0), -m_*rs_sqew(1,1), -m_*rs_sqew(1,2), 0, 0, 0, 0, 1, 0,
                    -m_*rs_sqew(2,0), -m_*rs_sqew(2,1), -m_*rs_sqew(2,2), 0, 0, 0, 0, 0, 1;
            H_ = Hc_matrix();
            Z_ = Z*x;
            R_ = Rf_matrix();
        }
        void zc_make(float ax,float ay, float az, float fx, float fy, float fz, float tx, float ty, float tz) {
            Eigen::MatrixXd x(9,1);
            x << ax, ay, az, fx, fy, fz, tx, ty, tz;
            Eigen::MatrixXd rs_sqew = rs_sqew_matrix();
            Eigen::MatrixXd Z(6,9);
            Z <<  -m_, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, -m_, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, -m_, 0, 0, 1, 0, 0, 0,
                    -m_*rs_sqew(0,0), -m_*rs_sqew(0,1), -m_*rs_sqew(0,2), 0, 0, 0, 1, 0, 0,
                    -m_*rs_sqew(1,0), -m_*rs_sqew(1,1), -m_*rs_sqew(1,2), 0, 0, 0, 0, 1, 0,
                    -m_*rs_sqew(2,0), -m_*rs_sqew(2,1), -m_*rs_sqew(2,2), 0, 0, 0, 0, 0, 1;
            Zc_ = Z*x;
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

        void uk(float fr, float ff, float fa) {
            Eigen::MatrixXd gw(3,1);
            gw << 0,0,-9.81;
            Eigen::MatrixXd gs  = Rws_.transpose()*gw;
            U = (gs - g_) * (fr / (ff - fa));
            g_ = gs;
        }

        void predict(float SKD, float t) {
            Q_calc(SKD,t);
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
        std::vector<std::string> z_variable_names;
        float m_;
        Eigen::MatrixXd x_; // State estimate
        Eigen::MatrixXd r_; // State estimate
        Eigen::MatrixXd P_; // State covariance
        Eigen::MatrixXd K_; // Kalman gain
        Eigen::MatrixXd Q; //
        Eigen::MatrixXd Z_; //
        Eigen::MatrixXd Zc_; //
        Eigen::MatrixXd H_; //
        Eigen::MatrixXd R_; //
        Eigen::MatrixXd U; //
        Eigen::MatrixXd B; // B
        Eigen::MatrixXd Rws_;
        Eigen::MatrixXd g_;
        std::vector<float> SaSDa_;
        std::vector<float> SfSDf_;
        std::vector<float> StSDt_;
        float prev_t;
    };
}

#endif
