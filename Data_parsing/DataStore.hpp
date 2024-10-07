//
// Created by joelo on 02.10.2024.
//

#include <rapidcsv.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#ifndef AIS4104_ASSIGNMENTS_DATASTORE_HPP
#define AIS4104_ASSIGNMENTS_DATASTORE_HPP


class DataStore {
public:
    DataStore(std::string csvFileName) : csvFileName_(csvFileName) {
        rapidcsv::Document data(csvFileName_);
        this->data_ = data;

}
    std::vector<std::string> parseCSVLine(const std::string& line) {
        std::vector<std::string> result;
        std::stringstream s_stream(line);
        std::string cell;
        while (std::getline(s_stream, cell, ',')) {
            result.push_back(cell);
        }
        return result;
    }

    std::vector<std::string> getColumnNamesVector() {
        return columnNames_;
    }

    std::string getColumnName(int index) {
        return columnNames_[index];
    }

    std::vector<float> getColumnByIdx(int index) {
        return data_.GetColumn<float>(index);
    }

    std::vector<float> getColumnByName(const std::string &name) {
        return data_.GetColumn<float>(name);
    }



    rapidcsv::Document getData() {
        return data_;
    }



    ~DataStore() {
        std::cout << "Object destructed" << std::endl;
    }

private:
    std::string csvFileName_ = {};
    std::vector<std::string> columnNames_ = {};
    rapidcsv::Document data_;
};


#endif //AIS4104_ASSIGNMENTS_DATASTORE_HPP
