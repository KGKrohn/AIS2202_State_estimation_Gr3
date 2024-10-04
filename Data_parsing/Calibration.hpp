//
// Created by joelo on 04.10.2024.
//
#include <rapidcsv.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#ifndef AIS4104_ASSIGNMENTS_CALIBRATION_HPP
#define AIS4104_ASSIGNMENTS_CALIBRATION_HPP

class CalibrationFTS {
public:
    explicit CalibrationFTS(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);
        this->variableNames_ = extraction.GetRow<std::string>(0);
        extraction.RemoveRow(0);
        for (int count = 0; count < extraction.GetColumnCount(); count++) {
            this->data_.push_back(extraction.GetColumn<float>(count));
        }
        this->fx_ = data_[getVariableNameIndex("fx")];
        this->fy_ = data_[getVariableNameIndex("fy")];
        this->fz_ = data_[getVariableNameIndex("fz")];
        this->tx_ = data_[getVariableNameIndex("tx")];
        this->ty_ = data_[getVariableNameIndex("ty")];
        this->tz_ = data_[getVariableNameIndex("tz")];
        this->ax_ = data_[getVariableNameIndex("ax")];
        this->ay_ = data_[getVariableNameIndex("ay")];
        this->az_ = data_[getVariableNameIndex("az")];
        this->gx_ = data_[getVariableNameIndex("gx")];
        this->gy_ = data_[getVariableNameIndex("gy")];
        this->gz_ = data_[getVariableNameIndex("gz")];
        this->r11_ = data_[getVariableNameIndex("r11")];
        this->r12_ = data_[getVariableNameIndex("r12")];
        this->r13_ = data_[getVariableNameIndex("r13")];
        this->r21_ = data_[getVariableNameIndex("r21")];
        this->r22_ = data_[getVariableNameIndex("r22")];
        this->r23_ = data_[getVariableNameIndex("23")];
        this->r31_ = data_[getVariableNameIndex("31")];
        this->r32_ = data_[getVariableNameIndex("r32")];
        this->r33_ = data_[getVariableNameIndex("33")];
    }

    int getVariableNameIndex(const std::string &name) {
        try{
            for (int i = 0; variableNames_.size() > i; i++) {
                if (name == variableNames_[i]){
                    return i;
                }
            }
            throw("Did not return a value");
        }
        catch(std::string &message) {
            std::cout << message << std::endl;
        }
    }

    std::vector<float> getForceVector(const int &index) {
        return {fx_[index], fy_[index], fz_[index]};
    }

    std::vector<std::vector<float>> getForceVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getTorqueVector(const int &index) {
        return {tx_[index], ty_[index], tz_[index]};
    }

    std::vector<std::vector<float>> getTorqueVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < tx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getAccVector(const int &index) {
        return {ax_[index], ay_[index], az_[index]};
    }

    std::vector<std::vector<float>> getAccVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < ax_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getGVector(const int &index) {
        return {gx_[index], gy_[index], gz_[index]};
    }

    std::vector<std::vector<float>> getGVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < gx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getR1Vector(const int &index) {
        return {r11_[index], r12_[index], r13_[index]};
    }

    std::vector<std::vector<float>> getR1VectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < r11_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getR2Vector(const int &index) {
        return {r21_[index], r22_[index], r23_[index]};
    }

    std::vector<std::vector<float>> getR2VectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < r21_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<float> getR3Vector(const int &index) {
        return {r31_[index], r32_[index], r33_[index]};
    }

    std::vector<std::vector<float>> getR3VectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < r31_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    Eigen::Matrix3f getRmatrix(const int &index) {
        Eigen::Matrix3f  m;
        m << r11_[index], r12_[index], r13_[index],
            r21_[index], r22_[index], r23_[index],
            r31_[index], r32_[index], r33_[index];
        return m;
    }

private:
    std::vector<float> fx_;
    std::vector<float> fy_;
    std::vector<float> fz_;
    std::vector<float> tx_;
    std::vector<float> ty_;
    std::vector<float> tz_;
    std::vector<float> ax_;
    std::vector<float> ay_;
    std::vector<float> az_;
    std::vector<float> gx_;
    std::vector<float> gy_;
    std::vector<float> gz_;
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
#endif //AIS4104_ASSIGNMENTS_CALIBRATION_HPP
