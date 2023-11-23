// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com

#include <iostream>
#include <memory>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <chrono>

#include <blaze/Blaze.h>

#include "KalmanFilter.hpp"

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &trajectoryName);

int main()
{
    // dump files for CTR calibration
    std::ofstream file;
    // If the file already exists, its previous content is deleted and replaced by the new one.
    file.open("../../InputFiles/Filtered_EM.dat", std::ios::out | std::ios::trunc);

    if (!file.is_open())
    {
        std::cerr << "Failed opening the trajectory dump file!\n";
        return 0;
    }

    // Container for storing the noisy EM sensor readings
    blaze::HybridMatrix<double, 10050UL, 15UL> EM_readings;

    // speficy which trajectory to consider
    std::string fileName("EM_Recordings");
    readFromCSV(EM_readings, fileName);
    const size_t numRows = EM_readings.rows();

    double dt = 0.025; // sampling period of the tracking system
    double q = 1.00;   // process variance 
    double r = 20.00;  // measurement variance 

    // Parameters for Kalman Filterring
    std::shared_ptr<KalmanFilter> KLF_tip = std::make_shared<KalmanFilter>(dt, q, r);

    // blaze::StaticVector<double, 3UL> target = blaze::StaticVector<double, 3UL>(0);
    std::vector<double> target(3UL, 0.00), filtered_State(3UL, 0.00);

    for (size_t row = 0UL; row < numRows; ++row)
    {
        target[0UL] = EM_readings(row, 8UL);
        target[1UL] = EM_readings(row, 9UL);
        target[2UL] = EM_readings(row, 10UL);

        KLF_tip->Loop(target, filtered_State);

        // Saving commanded joint values to file
        file << filtered_State[0UL] << ","
             << filtered_State[1UL] << ","
             << filtered_State[2UL] << "," 
             << target[0UL] << ","
             << target[1UL] << ","
             << target[2UL]
             << std::endl;
    }

    // close the dump file
    file.close();

    return 0;
}

// function that reads relevant clinical data from CSV files for each case
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &trajectoryName)
{
    std::string filePath, file;

    // relative path to the folder containing clinical data
    filePath = "../../InputFiles/";
    file = trajectoryName + ".csv";

    // file from which the information will be read from
    std::ifstream CSV_file;

    CSV_file.open((filePath + file).c_str(), std::ifstream::in);
    if (!CSV_file.is_open())
    {
        std::cerr << "Error opening the CSV file within: " << __PRETTY_FUNCTION__ << "\n";
        return Mat = -1.00;
    }

    typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

    std::string line;

    size_t row = 0UL, col = 0UL;
    double value;

    while (std::getline(CSV_file, line))
    {
        Tokenizer tokenizer(line);
        col = 0UL;

        for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
        {
            value = std::stod(*it);
            Mat(row, col) = value;
            ++col;
        }
        ++row;
    }

    CSV_file.close();
    Mat.resize(row, col, true);

    return Mat;
}