#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "kalman_filter.h"

inline double Square(double x)
{
    return x*x;
}

KalmanFilter::KalmanFilter(double predict_sigma, double measure_sigma)
{
    x_=0;
    u_=0;
    predict_sigma_=predict_sigma;
    measure_sigma_=measure_sigma;
    start_flag_=true;
}

KalmanFilter::~KalmanFilter()
{

}

double KalmanFilter::Refresh(double u, double measure_value)
{
    double prediction, prediction_sigma, kg;
    if(start_flag_)
    {
        x_=measure_value;
        sigma_=measure_sigma_;
        start_flag_=false;
    }
    else
    {
        prediction=x_+u;
        prediction_sigma=sqrt(Square(sigma_)+Square(predict_sigma_));
        kg=Square(prediction_sigma)/(Square(prediction_sigma)+Square(measure_sigma_));
        x_=prediction+kg*(measure_value-prediction);
        sigma_=sqrt((1.0-kg)*Square(prediction_sigma));
    }
    u_=u;
    return x_;
}
