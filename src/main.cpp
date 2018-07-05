#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include "kalman_filter.h"

double RandDouble(double min, double max)
{
    static bool start_flag=true;
    if(start_flag)
    {
        srand((int)time(0));
        start_flag=false;
    }
    double normalized_rand=(rand()%10000)/10000.0;
    return min+normalized_rand*(max-min);
}

double Normal(double x, double miu,double sigma)
{
    const double pi=3.1415926;
    return 1.0/sqrt(2*pi)/sigma*exp(-1*(x-miu)*(x-miu)/(2*sigma*sigma));
}

double RandNormal(double miu,double sigma)
{
    double x,y,dScope;
    do{
        x=RandDouble(miu-10*sigma,miu+10*sigma);
        y=Normal(x,miu,sigma);
        dScope=RandDouble(0.0,Normal(miu,miu,sigma));
    }while(dScope>y);
    return x;
}

//均值μ
double Mean(double *px, unsigned long data_num)
{
    double sum=0;
    for(unsigned long iloop=0;iloop<data_num;iloop++)
    {
        sum+=px[iloop];
    }
    return sum/data_num;
}

double MeanErr(double err)
{
    const int record_err_num=10;
    static double errs[record_err_num];
    static int err_num=0;
    static int err_pointer=0;

    errs[err_pointer]=err;
    err_pointer++;
    if(err_pointer>=record_err_num)
    {
        err_pointer=0;
    }
    if(err_num<record_err_num)
    {
        err_num++;
    }
    return Mean(errs, err_num);
}

int main(int argc, char** argv)
{
	double x_real=0;
    double x_measure;
    double speed=2.5;
    double speed_measure;
    double measure_sigma=10;
    double pridict_sigma=1;
    double x_estimate;
    double err;

    KalmanFilter kf(pridict_sigma, measure_sigma);
    printf("%10s%10s%10s%10s%10s\n", "x_measure", "x_real", "x_estimate", "err", "mean_err");
    printf("---------------------------------------------------------\n");
    for(int iloop=0;iloop<100;iloop++)
    {
        x_real+=speed;
        x_measure=RandNormal(x_real, measure_sigma);
        speed_measure=RandNormal(speed, pridict_sigma);
        x_estimate=kf.Refresh(speed_measure, x_measure);
        err=fabs(x_estimate-x_real);
        printf("%10.2f%10.2f%10.2f%10.2f%10.2f\n", 
            x_measure, 
            x_real, 
            x_estimate, 
            err,
            MeanErr(err));
        usleep(10000);
    }
	return 0;
}