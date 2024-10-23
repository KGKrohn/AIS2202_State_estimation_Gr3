#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>

namespace estimation {
    class kalman_filter {
    public:
        kalman_filter(const Eigen::MatrixXd& x0,
                      const Eigen::MatrixXd& P0,float m,
                      const Eigen::MatrixXd& r):
                      x_(x0), P_(P0), m_(m),r_(r),
                      Q(9,9),B(9,3),Rws_(3,3),g_(3,1) {};

        void update_static_variables(std::vector<float> SaSDa,
                                     std::vector<float> SfSDf,
                                     std::vector<float> StSDt) {
            SaSDa_ = SaSDa;
            SfSDf_ = SfSDf;
            StSDt_ = StSDt;
            Q_calc(0.5);
            B_calc();
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
            rs_sqew << 0, -r_(2,0), r_(1,0),
                    r_(2,0), 0, -r_(0,0),
                    -r_(1,0), r_(0,0), 0;
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

        Eigen::MatrixXd uk(float ax,float ay,float az, float fr, float ff, float fa) {
            Eigen::MatrixXd gw(3,1);
            gw << ax,ay,az;

            //Eigen::MatrixXd u_k = a * (fr / (ff - fa));
            Eigen::MatrixXd u_k = ((Rws_.transpose()*gw) - g_) * (fr / (ff - fa));
            g_ = (Rws_.transpose()*gw);
            return u_k;
        }

        void predict(Eigen::MatrixXd u) {
            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);

            x_ = A * x_ + B*u;
            P_ = A * P_ * A.transpose() + Q;
        }


        void update() {
            Eigen::MatrixXd z = Zc_matrix(x_);
            Eigen::MatrixXd H = Hf_matrix();
            Eigen::MatrixXd R = Rf_matrix();

            K_ = (P_ * H.transpose()) * (H * P_ * H.transpose() + R).inverse();
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
        /*
        Eigen::MatrixXd H_matrix() {
            Eigen::MatrixXd H(9,9);
            H <<    0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 1,
                    1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0;
            return H;
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
        */
        void Rws(float r11,float r12,float r13,float r21,float r22,float r23,float r31,float r32,float r33) {
            Rws_ << r11, r12, r13,
                    r21, r22, r23,
                    r31, r32, r33;
        }

    private:
        float m_;
        Eigen::MatrixXd x_; // State estimate
        Eigen::MatrixXd r_; // State estimate
        Eigen::MatrixXd P_; // State covariance
        Eigen::MatrixXd K_; // Kalman gain
        Eigen::MatrixXd Q; //
        Eigen::MatrixXd B; // B
        Eigen::MatrixXd Rws_;
        Eigen::MatrixXd g_;
        std::vector<float> SaSDa_;
        std::vector<float> SfSDf_;
        std::vector<float> StSDt_;
    };
}

#endif
