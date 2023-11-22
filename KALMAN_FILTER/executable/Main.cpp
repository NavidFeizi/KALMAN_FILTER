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
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#include "EMTracker.hpp"

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &trajectoryName);
std::vector<double> q2qRobot(const blaze::StaticVector<double, 6UL> &input);

int main()
{
    // dump files for CTR calibration
    std::ofstream file;
    // If the file already exists, its previous content is deleted and replaced by the new one.
    file.open("../../Trajectories/Filtered_EM_Trajectory.dat", std::ios::out | std::ios::trunc);

    if (!file.is_open())
    {
        std::cerr << "Failed opening the trajectory dump file!\n";
        return 0;
    }

    // Trajectory to be tracked by the control loop ==> Either a Helix or a Hypocycloid
    blaze::HybridMatrix<double, 10050UL, 15UL> Trajectory;

    // speficy which trajectory to consider
    std::string trajectory("EM_Recordings");
    readFromCSV(Trajectory, trajectory);
    const size_t numRows = Trajectory.rows();

    size_t row = 0UL;
    double elapsed_time, end_time, loop_time;
    std::chrono::high_resolution_clock::time_point start_time, current_time, loop_start_time;

    loop_time = 0.00;
    elapsed_time = 0.00;
    end_time = Trajectory(numRows - 1UL, 0UL);
    start_time = std::chrono::high_resolution_clock::now();

    // Parameters for Kalman Filterring
    std::shared_ptr<KalmanFilter> KLF_tip;
    std::shared_ptr<KalmanFilter> KLF_probe;
    KLF_tip = std::make_shared<KalmanFilter>(100, 0.01);
    // KLF_probe = std::make_shared<KalmanFilter>(1.00, 1.00);

    // blaze::StaticVector<double, 3UL> target = blaze::StaticVector<double, 3UL>(0);
    std::vector<double> target_flt(3, 0.0);
    std::vector<double> target(3, 0.0);

    for (size_t i = 0UL; i < numRows; ++i)
    {
        // updates the target based on trajectory time stamp
        loop_start_time = std::chrono::high_resolution_clock::now();
        if (elapsed_time >= Trajectory(row, 0UL))
        {
            target[0UL] = Trajectory(row, 8UL);
            target[1UL] = Trajectory(row, 9UL);
            target[2UL] = Trajectory(row, 10UL);
            row++;
        }

        KLF_tip->Loop(target, &target_flt);

        // Saving commanded joint values to file
        file << target_flt[0UL] << ","
                               << target_flt[1UL] << ","
                               << target_flt[2UL] << std::endl;

        current_time = std::chrono::high_resolution_clock::now();
        loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - loop_start_time).count();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
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
    filePath = "../../Trajectories/";
    file = trajectoryName + ".csv";

    // file from which the information will be read from
    std::ifstream CSV_file;
    std::cout << filePath + file << std::endl;
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