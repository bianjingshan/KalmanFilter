#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

class KalmanFilter{
public:
    KalmanFilter(double predict_sigma, double measure_sigma);
    ~KalmanFilter();
    double Refresh(double u, double measure_value);

private:
    double x_; 
    double u_; 
    double predict_sigma_; 
    double measure_sigma_; 
    double sigma_;
    bool start_flag_;
};


#endif



