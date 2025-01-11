#ifndef GLOBAL_CONFIG_HPP
#define GLOBAL_CONFIG_HPP

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
#include <stdexcept>
#include <memory>


using namespace std;

#define GAUSSIAN_FLAG 0
#define ADAPTIVE_FLAG 0
#define USE_DYNAMIC_STABLE_ERROR 0      //使用动态误差存在风险，后续误差会越来越小，导致到位时间更快，使适应度更倾向调整后的值

#define ALLOW_STABLE_IN_PLACE_ERROR 0.01
#define ALLOW_FIRST_IN_PLACE_ERROR 1.0
#define STABLE_COUNT 20

double car_position_init = 0.0;         // 小车初始位置
double car_position = 0.0;              // 小车实时位置
double target_pos = 300.0;              // 目标位置
double dt = 0.1;                        // 时间步长
int steps = 1000;                       // 仿真步数
double current_time;                    // 当前时间

namespace plt = matplotlibcpp;
vector<double> static_cur_positions;
vector<double> dynamic_cur_positions;
vector<double> target_positions;
vector<double> times;

//unique_ptr<vector<double>> tube_pos_data = make_unique<vector<double>>();

// vector<double> *tube_pos_data = new vector<double>;
// vector<double> tube_pos_data;

vector<double> dynamic_err(1e6);

vector<double> tube_pos_data(0);


#define MATPLOT_WIDE 2400
#define MATPLOT_HIGHT 600



#endif