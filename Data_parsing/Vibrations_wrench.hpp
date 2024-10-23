//
// Created by joelo on 14.10.2024.
//
#include <rapidcsv.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#ifndef AIS4104_ASSIGNMENTS_VIBRATIONS_WRENCH_HPP
#define AIS4104_ASSIGNMENTS_VIBRATIONS_WRENCH_HPP


class Vibrations_wrench {
public:
    Vibrations_wrench(const std::string &csvFile) {
        rapidcsv::Document extraction(csvFile);

        this->variableNames_ = extraction.GetColumnNames();

        for (int count = 0; count < variableNames_.size(); count++) {
            this->data_.push_back(extraction.GetColumn<float>(count));
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

    std::vector<float> getForceVector(const int &index) {
        return {fx_[index], fy_[index], fz_[index]};
    }

    std::vector<float> getForceMappedVector(const int &index) {
        return {fxmapped_[index], fymapped_[index], fzmapped_[index]};
    }

    std::vector<float> getForceWithTime(const int &index) {
        return {t_[index], fx_[index], fy_[index], fz_[index]};
    }

    std::vector<float> getTorqueVector(const int &index) {
        return {tx_[index], ty_[index], tz_[index]};
    }

    std::vector<float> getTorqueMappedVector(const int &index) {
        return {txmapped_[index], tymapped_[index], tzmapped_[index]};
    }

    std::vector<float> getAccTorqueWithTime(const int &index) {
        return {t_[index], tx_[index], ty_[index], tz_[index]};
    }

    std::vector<std::vector<float>> getForceVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getForceVectorWithTimeColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceWithTime(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getTorqueVectorColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceVector(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getTorqueVectorWithTimeColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fx_.size(); i++) {
            column.push_back(getForceWithTime(i));
        }
        return column;
    }

    void setVectorMapping(const float &sizeOfColumnWanted) {
        float interval = float(t_.size()) / sizeOfColumnWanted;
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            txmapped_.emplace_back(tx_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            tymapped_.emplace_back(ty_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            tzmapped_.emplace_back(tz_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            fxmapped_.emplace_back(fx_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            fymapped_.emplace_back(fy_[int(std::round(float(i) * interval))]);
        }
        for(int i = 0; i < sizeOfColumnWanted; i++) {
            fzmapped_.emplace_back(fz_[int(std::round(float(i) * interval))]);
        }
    }

    std::vector<std::vector<float>> getTorqueVectorMappingColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < txmapped_.size(); i++) {
            column.push_back(getForceMappedVector(i));
        }
        return column;
    }

    std::vector<std::vector<float>> getForceVectorMappingColumn() {
        std::vector<std::vector<float>> column = {};
        for (int i = 0; i < fxmapped_.size(); i++) {
            column.push_back(getForceMappedVector(i));
        }
        return column;
    }

    std::vector<float> getSingleTypeColumn_t_() {
        return t_;
    }

    std::vector<float> getSingleTypeColumn_fx_() {
        return fx_;
    }

    std::vector<float> getSingleTypeColumn_fy_() {
        return fy_;
    }

    std::vector<float> getSingleTypeColumn_fz_() {
        return fz_;
    }

    std::vector<float> getSingleTypeColumn_tx_() {
        return tx_;
    }

    std::vector<float> getSingleTypeColumn_ty_() {
        return ty_;
    }

    std::vector<float> getSingleTypeColumn_tz_() {
        return tz_;
    }
    std::vector<float> getSingleTypeColumnMapped_fx_() {
        return fxmapped_;
    }

    std::vector<float> getSingleTypeColumnMapped_fy_() {
        return fymapped_;
    }

    std::vector<float> getSingleTypeColumnMapped_fz_() {
        return fzmapped_;
    }

    std::vector<float> getSingleTypeColumnMapped_tx_() {
        return txmapped_;
    }

    std::vector<float> getSingleTypeColumnMapped_ty_() {
        return tymapped_;
    }

    std::vector<float> getSingleTypeColumnMapped_tz_() {
        return tzmapped_;
    }

private:
    std::vector<float> t_;
    std::vector<float> fx_;
    std::vector<float> fy_;
    std::vector<float> fz_;
    std::vector<float> tx_;
    std::vector<float> ty_;
    std::vector<float> tz_;
    std::vector<float> fxmapped_{};
    std::vector<float> fymapped_{};
    std::vector<float> fzmapped_{};
    std::vector<float> txmapped_{};
    std::vector<float> tymapped_{};
    std::vector<float> tzmapped_{};
    std::vector<std::string> variableNames_;
    std::vector<std::vector<float>> data_;

};


#endif //AIS4104_ASSIGNMENTS_VIBRATIONS_WRENCH_HPP
