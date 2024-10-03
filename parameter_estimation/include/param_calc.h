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
                       std::vector<std::vector<float>> t,
                       std::vector<std::vector<float>> r)
    {
        F = conv_vector_to_72x1_matrix(f);
        G = conv_vector_to_72x1_matrix(g);
        T = conv_vector_to_72x1_matrix(t);
        A = define_A(g);
        R = conv_vector_to_72x3_matrix(r);
    }

    static Eigen::MatrixXd conv_vector_to_72x1_matrix(std::vector<std::vector<float>> vector)
    {
        Eigen::MatrixXd Matrix(72, 1);
        for (int i = 0; i < 24; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                Matrix(i, 0) = vector[i][j];
            }
        }
        return Matrix;
    }
    static Eigen::MatrixXd conv_vector_to_72x3_matrix(std::vector<std::vector<float>> vector)
    {
        Eigen::MatrixXd Matrix(72, 3);
        for (int i = 0; i < 72; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                Matrix(i, j) = vector[i][j];
            }
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
            A(i*3+ 1 , 0) = gz;
            A(i*3+ 1 , 1) = 0;
            A(i*3+ 1 , 2) = -gx;
            A(i*3+2 , 0) = -gy;
            A(i*3+2 , 1) = gx;
            A(i*3+2 , 2) = 0;
        }

        return A;
    }


    [[nodiscard]] float m_calc()
    {
        auto v1 =(G.transpose()*F);
        auto v2 = (G.transpose()*G);
        float m =v1.value()/v2.value();
        return m;
    }


    [[nodiscard]] Eigen::MatrixXd  r_calc()
    {
        auto r = 1 / m_calc() * (A.transpose()*A).inverse() * A.transpose() * T;

        return r;
    }




private:
    Eigen::MatrixXd F;
    Eigen::MatrixXd G;
    Eigen::MatrixXd T;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
};



#endif //AIS4104_ASSIGNMENTS_PARAM_CALC_H
