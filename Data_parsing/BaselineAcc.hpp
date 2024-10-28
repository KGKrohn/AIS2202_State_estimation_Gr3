//
// Created by joelo on 14.10.2024.
//



#include <rapidcsv.h>
#include <vector>
#include <string>
#include <stdexcept>

#ifndef AIS4104_ASSIGNMENTS_BASELINEACC_HPP
#define AIS4104_ASSIGNMENTS_BASELINEACC_HPP

class BaselineAcc {
public:
    BaselineAcc(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);

        this->variableNames_ = extraction.GetColumnNames();

        for (int count = 0; count < variableNames_.size(); count++) {
            this->data_.push_back(extraction.GetColumn<double>(count));
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

    std::vector<double> getAccelerationVector(const int &index) {
        return {ax_[index], ay_[index], az_[index]};
    }

    std::vector<double> getAccelerationWithTime(const int &index) {
        return {t_[index], ax_[index], ay_[index], az_[index]};
    }

    std::vector<std::vector<double>> getAccelerationVectorColumn() {
        std::vector<std::vector<double>> column;
        for (int i = 0; i < ax_.size(); i++) {
            column.push_back(getAccelerationVector(i));
        }
        return column;
    }

    std::vector<std::vector<double>> getAccelerationVectorWithTimeColumn() {
        std::vector<std::vector<double>> column;
        for (int i = 0; i < ax_.size(); i++) {
            column.push_back(getAccelerationWithTime(i));
        }
        return column;
    }

    std::vector<double> getSingleTypeColumn_t() const {
        return t_;
    }

    std::vector<double> getSingleTypeColumn_ax() const {
        return ax_;
    }

    std::vector<double> getSingleTypeColumn_ay() const {
        return ay_;
    }

    std::vector<double> getSingleTypeColumn_az() const {
        return az_;
    }

private:
    std::vector<double> t_;
    std::vector<double> ax_;
    std::vector<double> ay_;
    std::vector<double> az_;
    std::vector<std::string> variableNames_;
    std::vector<std::vector<double>> data_;
};

#endif//AIS4104_ASSIGNMENTS_BASELINEACC_HPP
