#ifndef AIS4104_ASSIGNMENTS_PARAM_CALC_H
#define AIS4104_ASSIGNMENTS_PARAM_CALC_H
#include "Eigen/Dense"
#include "vector"

class parameter_calculation
{
public:

    parameter_calculation():F(72,1),G(72,1),R(72,3),T(72,1), A(72,3){}

    void update_Matrix(std::vector<std::vector<float>> f,
                       std::vector<std::vector<float>> g,
                       std::vector<std::vector<float>> t)
    {
        F = conv_vector_to_72x1_matrix(f);
        G = conv_vector_to_72x1_matrix(g);
        T = conv_vector_to_72x1_matrix(t);
        A = define_A(g);
    }

    static Eigen::MatrixXd conv_vector_to_72x1_matrix(std::vector<std::vector<float>> vector)
    {
        Eigen::MatrixXd Matrix(72, 1);
        for (int i = 0; i < 24; i++)
        {
            Matrix(i*3, 0) = vector[i][0];
            Matrix(i*3+1, 0) = vector[i][1];
            Matrix(i*3+2, 0) = vector[i][2];
        }
        return Matrix;
    }

    [[nodiscard]] Eigen::MatrixXd  define_A(std::vector<std::vector<float>> g)
    {
        for (int i = 0; i < 24; i++)
        {
            float gx = g[i][0]; // float gx
            float gy = g[i][1]; // float gy
            float gz = g[i][2]; // float gz

            A(i*3  , 0) = 0;
            A(i*3 , 1) = gz;
            A(i*3 , 2) = -gy;
            A(i*3+ 1 , 0) = -gz;
            A(i*3+ 1 , 1) = 0;
            A(i*3+ 1 , 2) = gx;
            A(i*3+2 , 0) = gy;
            A(i*3+2 , 1) = -gx;
            A(i*3+2 , 2) = 0;
        }
        return A;
    }
    void update_R(std::vector<std::vector<float>> R1,
                  std::vector<std::vector<float>> R2,
                  std::vector<std::vector<float>> R3)
    {
        for (int i = 0; i < 24; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                R(i*3 ,j) = R1[i][j];
                R(i*3+1 ,j) = R2[i][j];
                R(i*3+2 ,j) = R3[i][j];
            }
        }
    }


    float m_calc()
    {

        Eigen::VectorXd Fv(Eigen::Map<Eigen::VectorXd>(F.data(), F.cols() * F.rows()));
        Eigen::VectorXd Gv(Eigen::Map<Eigen::VectorXd>(G.data(), G.cols() * G.rows()));
        float v1 = Gv.transpose().dot(Fv);
        float v2 = Gv.transpose().dot(Gv);

        float m =v1/v2;
        return m;
    }
    Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
    {
        return A.completeOrthogonalDecomposition().pseudoInverse();
    }

    [[nodiscard]] Eigen::MatrixXd  r_calc()
    {
        auto r = 1 / m_calc() * pseudo_inverse((A.transpose()*A)) * A.transpose() * T;

        return r;
    }

    [[nodiscard]] Eigen::MatrixXd  fg_calc()
    {
        Eigen::MatrixXd f(6,1);
        Eigen::VectorXd f1(3,1);
        Eigen::VectorXd f2(3,1);
        Eigen::Vector3d  r = r_calc();
        Eigen::Vector3d u1;
        Eigen::Vector3d u2;

        u1 =  R.transpose()*G;
        f1 = m_calc()*u1;

        u2 = r.cross(u2);
        f2 = m_calc()*u2;

        f << f1(0,0),f1(1,0),f1(2,0), f2(0,0),f2(1,0),f2(2,0);
        return f;
    }



private:
    Eigen::MatrixXd F;
    Eigen::MatrixXd G;
    Eigen::MatrixXd T;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
};



#endif //AIS4104_ASSIGNMENTS_PARAM_CALC_H
