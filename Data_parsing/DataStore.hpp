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
    DataStore(std::string &csvFileName) : csvFileName_(csvFileName) {
        std::ifstream file(csvFileName);
        if (!file.is_open()) {
            std::cerr << "Could not open the file!" << std::endl;
        }

        std::string line;
        std::vector<std::string> columnNames;
        bool isFirstRow = true;

        while (std::getline(file, line)) {
            std::vector<std::string> row = parseCSVLine(line);

            if (isFirstRow) {
                columnNames_ = row;
                isFirstRow = false;
            } else {
            std::vector<float> floatRow;
            for (const std::string& value : row) {
                try {
                    floatRow.push_back(std::stof(value));
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid data: Unable to convert '" << value << "' to float." << std::endl;
                } catch (const std::out_of_range& e) {
                    std::cerr << "Out of range: Value '" << value << "' is too large to convert to float." << std::endl;
                }
            }
            data_.push_back(floatRow);
        }
    }

    file.close();
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



    ~DataStore() {
        std::cout << "Object destructed" << std::endl;
    }

private:
    std::string csvFileName_ = {};
    std::vector<std::string> columnNames_ = {};
    std::vector<std::vector<float>> data_ = {};
};


#endif //AIS4104_ASSIGNMENTS_DATASTORE_HPP
