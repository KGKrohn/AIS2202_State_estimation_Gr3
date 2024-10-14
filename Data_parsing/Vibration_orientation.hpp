//
// Created by joelo on 14.10.2024.
//
#include <rapidcsv.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#ifndef AIS4104_ASSIGNMENTS_VIBRATION_ORIENTATION_HPP
#define AIS4104_ASSIGNMENTS_VIBRATION_ORIENTATION_HPP


class Vibration_orientation {
public:
    explicit Vibration_orientation(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);

        this->variableNames_ = extraction.GetColumnNames();

        for (int count = 0; count < variableNames_.size(); count++) {
            this->data_.push_back(extraction.GetColumn<float>(count));
        }

        this->t_ = data_[getVariableNameIndex("t")];
        this->r11_ = data_[getVariableNameIndex("r11")];
        this->r12_ = data_[getVariableNameIndex("r12")];
        this->r13_ = data_[getVariableNameIndex("r13")];
        this->r21_ = data_[getVariableNameIndex("r21")];
        this->r22_ = data_[getVariableNameIndex("r22")];
        this->r23_ = data_[getVariableNameIndex("r23")];
        this->r31_ = data_[getVariableNameIndex("r31")];
        this->r32_ = data_[getVariableNameIndex("r32")];
        this->r33_ = data_[getVariableNameIndex("r33")];
    }


    int getVariableNameIndex(const std::string &name) {
        for (int i = 0; i < variableNames_.size(); i++) {
            if (name == variableNames_[i]) {
                return i;
            }
        }
        throw std::runtime_error("Variable name not found: " + name);
    }

    std::vector<float> getRVector(const int &index) {
        return {r11_[index], r12_[index], r13_[index], r21_[index], r22_[index], r23_[index], r31_[index], r32_[index], r33_[index]};
    }

    std::vector<float> getRVectorWithTime(const int &index) {
        return {t_[index], r11_[index], r12_[index], r13_[index], r21_[index], r22_[index], r23_[index], r31_[index], r32_[index], r33_[index]};
    }

    std::vector<std::vector<float>> getRVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < r11_.size(); i++) {
            column.push_back(getRVector(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getAccVectorWithTimeColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < r11_.size(); i++) {
            column.push_back(getRVectorWithTime(i));
        }
        return column;
    }

    std::vector<float> getSingleTypeColumn_t_() {
        return t_;
    }

    std::vector<float> getSingleTypeColumn_r11_() {
        return r11_;
    }

    std::vector<float> getSingleTypeColumn_r12_() {
        return r12_;
    }

    std::vector<float> getSingleTypeColumn_r13_() {
        return r13_;
    }

    std::vector<float> getSingleTypeColumn_r21_() {
        return r21_;
    }

    std::vector<float> getSingleTypeColumn_r22_() {
        return r22_;
    }

    std::vector<float> getSingleTypeColumn_r23_() {
        return r23_;
    }

    std::vector<float> getSingleTypeColumn_r31_() {
        return r31_;
    }

    std::vector<float> getSingleTypeColumn_r32_() {
        return r32_;
    }

    std::vector<float> getSingleTypeColumn_r33_() {
        return r33_;
    }

    Eigen::Matrix3f getRmatrix(const int &index) {
        Eigen::Matrix3f  m;
        m << r11_[index], r12_[index], r13_[index],
                r21_[index], r22_[index], r23_[index],
                r31_[index], r32_[index], r33_[index];
        return m;
    }

    std::vector<Eigen::Matrix3f> getRMatrixColumn() {
        std::vector<Eigen::Matrix3f> column = {};
        for (int i = 0; i < r31_.size(); i++) {
            column.push_back(getRmatrix(i));
        }
        return column;
    }

private:
    std::vector<float> t_;
    std::vector<float> r11_;
    std::vector<float> r12_;
    std::vector<float> r13_;
    std::vector<float> r21_;
    std::vector<float> r22_;
    std::vector<float> r23_;
    std::vector<float> r31_;
    std::vector<float> r32_;
    std::vector<float> r33_;
    std::vector<std::string> variableNames_;
    std::vector<std::vector<float>> data_;
};


#endif //AIS4104_ASSIGNMENTS_VIBRATION_ORIENTATION_HPP
