//
// Created by joelo on 14.10.2024.
//
#include <rapidcsv.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#ifndef AIS2202_STATE_ESTIMATION_GR3_VIBRATION_ACC_HPP
#define AIS2202_STATE_ESTIMATION_GR3_VIBRATION_ACC_HPP


class Vibration_Acc {
public:
    explicit Vibration_Acc(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);

        this->variableNames_ = extraction.GetColumnNames();

        for (int count = 0; count < variableNames_.size(); count++) {
            this->data_.push_back(extraction.GetColumn<float>(count));
        }

        this->t_ = data_[getVariableNameIndex("t")];
        this->ax_ = data_[getVariableNameIndex("ax")];
        this->ay_ = data_[getVariableNameIndex("ay")];
        this->az_ = data_[getVariableNameIndex("az")];
    }


    int getVariableNameIndex(const std::string &name) {
        for (int i = 0; i < variableNames_.size(); i++) {
            if (name == variableNames_[i]) {
                return i;
            }
        }
        throw std::runtime_error("Variable name not found: " + name);
    }

    std::vector<float> getAccVector(const int &index) {
        return {ax_[index], ay_[index], az_[index]};
    }

    std::vector<float> getAccelerationMappedVector(const int &index) {
        return {axmapped_[index], aymapped_[index], azmapped_[index]};
    }

    std::vector<float> getAccVectorWithTime(const int &index) {
        return {t_[index], ax_[index], ay_[index], az_[index]};
    }

    std::vector<std::vector<float>> getAccVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < ax_.size(); i++) {
            column.push_back(getAccVector(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getAccVectorWithTimeColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < ax_.size(); i++) {
            column.push_back(getAccVectorWithTime(i));
        }
        return column;
    }

    void setVectorMapping(const float &sizeOfColumnWanted) {
        float interval = float(t_.size()) / sizeOfColumnWanted;
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            axmapped_.emplace_back(ax_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            aymapped_.emplace_back(ay_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            azmapped_.emplace_back(az_[int(std::round(float(i) * interval))]);
        }
    }

    std::vector<std::vector<float>> getAccVectorMappingColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < axmapped_.size(); i++) {
            column.push_back(getAccelerationMappedVector(i));
        }
        return column;
    }

    std::vector<float> getSingleTypeColumn_t_() {
        return t_;
    }

    std::vector<float> getSingleTypeColumn_ax_() {
        return ax_;
    }

    std::vector<float> getSingleTypeColumn_ay_() {
        return ay_;
    }

    std::vector<float> getSingleTypeColumn_az_() {
        return az_;
    }

    std::vector<float> getSingleTypeMappedColumn_ax() const {
        return axmapped_;
    }

    std::vector<float> getSingleTypeMappedColumn_ay() const {
        return aymapped_;
    }

    std::vector<float> getSingleTypeMappedColumn_az() const {
        return azmapped_;
    }

private:
    std::vector<float> t_;
    std::vector<float> ax_;
    std::vector<float> ay_;
    std::vector<float> az_;
    std::vector<float> axmapped_{};
    std::vector<float> aymapped_{};
    std::vector<float> azmapped_{};
    std::vector<std::string> variableNames_;
    std::vector<std::vector<float>> data_;
};
#endif //AIS2202_STATE_ESTIMATION_GR3_VIBRATION_ACC_HPP
