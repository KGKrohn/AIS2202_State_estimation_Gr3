//
// Created by joelo on 14.10.2024.
//
#include <rapidcsv.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#ifndef AIS4104_ASSIGNMENTS_BASELINE_WRENCH_HPP
#define AIS4104_ASSIGNMENTS_BASELINE_WRENCH_HPP


class Baseline_wrench {
public:
    Baseline_wrench(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);

        this->variableNames_ = extraction.GetColumnNames();

        for (int count = 0; count < variableNames_.size(); count++) {
            this->data_.push_back(extraction.GetColumn<double>(count));
        }

        this->t_ = data_[getVariableNameIndex("t")];
        this->fx_ = data_[getVariableNameIndex("fx")];
        this->fy_ = data_[getVariableNameIndex("fy")];
        this->fz_ = data_[getVariableNameIndex("fz")];
        this->tx_ = data_[getVariableNameIndex("tx")];
        this->ty_ = data_[getVariableNameIndex("ty")];
        this->tz_ = data_[getVariableNameIndex("tz")];
    }


    int getVariableNameIndex(const std::string &name) {
        for (int i = 0; i < variableNames_.size(); i++) {
            if (name == variableNames_[i]) {
                return i;
            }
        }
        throw std::runtime_error("Variable name not found: " + name);
    }

    std::vector<double> getForceVector(const int &index) {
        return {fx_[index], fy_[index], fz_[index]};
    }

    std::vector<double> getForceWithTime(const int &index) {
        return {t_[index], fx_[index], fy_[index], fz_[index]};
    }

    std::vector<double> getTorqueVector(const int &index) {
        return {tx_[index], ty_[index], tz_[index]};
    }

    std::vector<double> getAccTorqueWithTime(const int &index) {
        return {t_[index], tx_[index], ty_[index], tz_[index]};
    }

    std::vector<std::vector<double>> getForceVectorColumn() {
        std::vector<std::vector<double>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<std::vector<double>> getForceVectorWithTimeColumn() {
        std::vector<std::vector<double>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceWithTime(i));
        }
        return column;
    }


    std::vector<std::vector<double>> getTorqueVectorColumn() {
        std::vector<std::vector<double>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<std::vector<double>> getTorqueVectorWithTimeColumn() {
        std::vector<std::vector<double>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceWithTime(i));
        }
        return column;
    }

    std::vector<double> getSingleTypeColumn_t_() {
        return t_;
    }

    std::vector<double> getSingleTypeColumn_fx_() {
        return fx_;
    }

    std::vector<double> getSingleTypeColumn_fy_() {
        return fy_;
    }

    std::vector<double> getSingleTypeColumn_fz_() {
        return fz_;
    }

    std::vector<double> getSingleTypeColumn_tx_() {
        return tx_;
    }

    std::vector<double> getSingleTypeColumn_ty_() {
        return ty_;
    }

    std::vector<double> getSingleTypeColumn_tz_() {
        return tz_;
    }

private:
    std::vector<double> t_;
    std::vector<double> fx_;
    std::vector<double> fy_;
    std::vector<double> fz_;
    std::vector<double> tx_;
    std::vector<double> ty_;
    std::vector<double> tz_;
    std::vector<std::string> variableNames_;
    std::vector<std::vector<double>> data_;

};


#endif //AIS4104_ASSIGNMENTS_BASELINE_WRENCH_HPP
