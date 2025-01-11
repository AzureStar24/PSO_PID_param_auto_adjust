
#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include "matplotlibcpp.h"
#include <cstdio>
#include <chrono>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>
#include <memory>
#include "./key_value.h"
#include <stdexcept>
#include "global_config.hpp"



class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : prev_error(0.0), prev_prev_error(0.0)
    {
        proportion = 0.0;
        integral = 0.0;
        derivative = 0.0;
        increment = 0.0;
    }


    double position_control(double error, double dt)
    {
        proportion = error;
        // 积分的累加
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;
        // printf("PID param proportion = %f , integral = %f , derivative = %f \n", proportion, integral, derivative);
        return pid.K_P * proportion + pid.K_I * integral + pid.K_D * derivative;
    }

    double increment_control(double error, double dt)
    {

        // printf(" proportion = %f , error = %f , prev_error = %f , prev_prev_error = %f , pid.K_I = %f \n", \
            proportion, error, prev_error, prev_prev_error, pid.K_I);

        proportion = error - prev_error;
        integral = error * dt;
        derivative = (error - 2.0 * prev_error + prev_prev_error) / dt;
        
        // u(t) = u(t-1) + delta_u(t) 增量的累加
        increment += pid.K_P * proportion + pid.K_I * integral + pid.K_D * derivative;
        prev_prev_error = prev_error;
        prev_error = error;

        return increment;
    }

    void set_pid_param(double kp, double ki, double kd)
    {
        pid.K_P = kp;
        pid.K_I = ki;
        pid.K_D = kd;
    }




    struct PID{
        double K_P;
        double K_I;
        double K_D;
    };
    struct PID pid;


public:
    double prev_error, prev_prev_error;
    double proportion, integral, derivative;

    double increment; //增量PID的结果累计

};






class AdaptivePIDController {
public:
    AdaptivePIDController(double kp, double ki, double kd, double gamma)
        : pid_(kp, ki, kd), gamma_(gamma), theta_(0.0) {}

    double control(double error, double dt, double noise) {
        double pid_output = pid_.position_control(error, dt);
        double adaptive_output = adaptiveControl(error, noise);
        return pid_output + adaptive_output;
    }

private:
    PIDController pid_;
    double gamma_;
    double theta_;

    double adaptiveControl(double error, double noise) {
        theta_ -= gamma_ * noise * error;
        return theta_ * noise;
    }

};



AdaptivePIDController adaptive_pid(5.0, 10.0, 0.05, 0.01);
PIDController pid_class(5.0, 0.5, 0.0);



#endif