#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>

#include "KalmanFilter.hpp"

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &fileName);

// function that computes the covariance matrix of a set of EM data readings
template<size_t N>
blaze::StaticMatrix<double, 3UL, 3UL> computeCovarianceMatrix(const blaze::HybridMatrix<double, N, 3UL> &Mat);

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
    const size_t maxRows = 10050UL;
    blaze::HybridMatrix<double, maxRows, 15UL> EM_readings;
    blaze::HybridMatrix<double, maxRows, 3UL> EM_distalReadings;

    // speficy which file to read
    std::string fileName("EM_Recordings");
    // reads the file and stores data in memory
    readFromCSV(EM_readings, fileName);

    // number of data points in the data file (raw measurements to be filtered)
    const size_t numRows = EM_readings.rows();

    // matrix containing the EM readings at the distal end of the robot
    EM_distalReadings = blaze::submatrix( EM_readings, 0UL, 8UL, numRows, 3UL);

    std::cout << "EM_distalReadings.rows(): " << EM_distalReadings.rows() << " EM_distalReadings.columns(): " << EM_distalReadings.columns() << std::endl
              << "EM_readings.rows(): " << EM_readings.rows() << " EM_readings.columns(): " << EM_readings.columns() << std::endl;

    // Kalman filter parameters
    double dt = 0.025; // sampling period of the tracking system
    // process variance
    blaze::StaticMatrix<double, 6UL, 6UL> Q;
    // measurement variance
    blaze::StaticMatrix<double, 3UL, 3UL> R;

    blaze::diagonal(Q) = 15.00;
    R = computeCovarianceMatrix( EM_distalReadings );

    // Instantiating the Kalman Filter
    std::shared_ptr<KalmanFilter> KLF_tip = std::make_shared<KalmanFilter>(dt, Q, R);

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

    std::cout << "Covariance of the measurements: \n" << R << std::endl;
    

    return 0;
}

// function that computes the covariance matrix of a set of EM data readings
template<size_t N>
blaze::StaticMatrix<double, 3UL, 3UL> computeCovarianceMatrix(const blaze::HybridMatrix<double, N, 3UL> &Mat)
{
    // Calculate the mean of each dimension
    blaze::StaticVector<double, 3UL, blaze::rowVector> mean = {blaze::mean(blaze::column<0UL>(Mat)),
                                                               blaze::mean(blaze::column<1UL>(Mat)),
                                                               blaze::mean(blaze::column<2UL>(Mat))};
    
    // computing the deviation matrix
    blaze::HybridMatrix<double, N, 3UL> D(Mat);

    // Subtract the mean in-place to get the deviation matrix
    for (size_t row = 0UL; row < Mat.rows(); ++row) {
        blaze::row(D, row) -= mean;
    }

    blaze::StaticMatrix<double, 3UL, 3UL> Cov = blaze::trans(D) * D / (Mat.rows() - 1);

    return Cov;    
}

// function that reads relevant clinical data from CSV files for each case
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &fileName)
{
    std::string filePath, file;

    // relative path to the folder containing clinical data
    filePath = "../../InputFiles/";
    file = fileName + ".csv";

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