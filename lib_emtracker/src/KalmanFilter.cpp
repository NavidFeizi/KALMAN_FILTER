#include "KalmanFilter.hpp"
#include <iostream>

// overloaded constructor
KalmanFilter::KalmanFilter(const double delta_t, const double q, const double r) : I(6UL)
{
    // Sampling period of the NDI tracking system
    dt = delta_t;

    // dynamics of the filter -- Nearly constant speed model
    A = {{1.00, dt, 0.00, 0.00, 0.00, 0.00},
         {0.00, 1.00, 0.00, 0.00, 0.00, 0.00},
         {0.00, 0.00, 1.00, dt, 0.00, 0.00},
         {0.00, 0.00, 0.00, 1.00, 0.00, 0.00},
         {0.00, 0.00, 0.00, 0.00, 1.00, dt},
         {0.00, 0.00, 0.00, 0.00, 0.00, 1.00}};

    // output matrix
    C = {{1.00, 0.00, 0.00, 0.00, 0.00, 0.00},
         {0.00, 0.00, 1.00, 0.00, 0.00, 0.00},
         {0.00, 0.00, 0.00, 0.00, 1.00, 0.00}};

    // initial state
    x = 1.00;
    blaze::diagonal(Q) = q; // covariance of the process noise 
    blaze::diagonal(R) = r; // covariance of the measurements

    // initial corariance
    Pxx = A * x * blaze::trans(x) * blaze::trans(A) + Q;
}

// KalmanFilter desctructor
KalmanFilter::~KalmanFilter()
{
    // nothing to be done
}

// copy constructor
KalmanFilter::KalmanFilter(const KalmanFilter &rhs) : dt(rhs.dt), A(rhs.A), C(rhs.C), x(rhs.x), y(rhs.y), y_hat(rhs.y_hat), Pxx(rhs.Pxx), I(rhs.I), W(rhs.W), S_inv(rhs.S_inv), Q(rhs.Q), R(rhs.R){};

// move constructor
KalmanFilter::KalmanFilter(KalmanFilter &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = std::move(rhs.A);
        this->C = std::move(rhs.C);
        this->x = std::move(rhs.x);
        this->y = std::move(rhs.y);
        this->y_hat = std::move(rhs.y_hat);
        this->Pxx = std::move(rhs.Pxx);
        this->I = std::move(rhs.I);
        this->W = std::move(rhs.W);
        this->S_inv = std::move(rhs.S_inv);
        this->Q = std::move(rhs.Q);
        this->R = std::move(rhs.R);
    }
}

// Copy assignment operator
KalmanFilter &KalmanFilter::operator=(const KalmanFilter &rhs)
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = rhs.A;
        this->C = rhs.C;
        this->x = rhs.x;
        this->y = rhs.y;
        this->y_hat = rhs.y_hat;
        this->Pxx = rhs.Pxx;
        this->I = rhs.I;
        this->W = rhs.W;
        this->S_inv = rhs.S_inv;
        this->Q = rhs.Q;
        this->R = rhs.R;
    }

    return *this;
}

// move assignment operator
KalmanFilter &KalmanFilter::operator=(KalmanFilter &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = std::move(rhs.A);
        this->C = std::move(rhs.C);
        this->x = std::move(rhs.x);
        this->y = std::move(rhs.y);
        this->y_hat = std::move(rhs.y_hat);
        this->Pxx = std::move(rhs.Pxx);
        this->I = std::move(rhs.I);
        this->W = std::move(rhs.W);
        this->S_inv = std::move(rhs.S_inv);
        this->Q = std::move(rhs.Q);
        this->R = std::move(rhs.R);
    }

    return *this;
}

// update core of the Kalman filter
void KalmanFilter::Loop(std::vector<double> &measurement, std::vector<double> &estimate)
{   
    // predicted measurement
    y_hat = C * x;

    // measurement covariance
    S_inv = blaze::inv(C * Pxx * blaze::trans(C) + R);

    // Kalman Gain
    W = Pxx * blaze::trans(C) * S_inv;

    // acquiring the measurements
    y[0UL] = measurement[0UL];
    y[1UL] = measurement[1UL];
    y[2UL] = measurement[2UL];  

    // updating the state estimates ==> filtered state
    x = x + W * (y - y_hat);

    // returning the current filtered state
    estimate[0UL] = x[0UL];
    estimate[1UL] = x[2UL];
    estimate[2UL] = x[4UL];

    // updating the state covariance
    Pxx = (I - W * C) * Pxx;

    // next state prediction
    x = A * x;

    // predicted state covariance
    Pxx = A * Pxx * blaze::trans(A) + Q;
}