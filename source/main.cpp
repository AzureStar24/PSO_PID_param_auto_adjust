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
#include "./key_value.h"
#include <stdexcept>
#include "pid.hpp"
#include "global_config.hpp"
#include "PSO.hpp"





//高斯噪声
double generateGaussianNoise(double mean, double stddev) {
    static std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> dist(mean, stddev);
    return dist(gen);
}










int main()
{
    bool use_gaussian_noise = true;  // 是否使用高斯噪声
    double noise = 0.0;
    string pid_p, pid_i, pid_d;
    
    /*
        init 参数初始化
    */
    pid_p = getKeyValue("../map.txt", "pid_p", "level3");
    pid_i = getKeyValue("../map.txt", "pid_i", "level0");
    pid_d = getKeyValue("../map.txt", "pid_d", "level0");

    printf(" init pid_p = %s , pid_i = %s , pid_d = %s \n", pid_p.c_str(), pid_i.c_str(), pid_d.c_str());
    printf(" init stod(pid_p) = %f , stod(pid_i) = %f , stod(pid_d) = %f \n", stod(pid_p), stod(pid_i), stod(pid_d));
    
    pid_class.pid.K_P = stod(pid_p);
    pid_class.pid.K_I = stod(pid_i);
    pid_class.pid.K_D = stod(pid_d);

    pid_class.set_pid_param(stod(pid_p), stod(pid_i), stod(pid_d));

    system("echo "" > ../performance_param.txt");

    //tube_pos_data.resize(1000);
    car_position = car_position_init;
    static_cur_positions.push_back(car_position);
    dynamic_cur_positions.push_back(car_position);
    performance_index.Servo_Performance_init();
    performance_index.Servo_Performance_calibration_init(car_position, target_pos);

    std::vector<double> pos_gbest;
    std::vector<double> pos_zbest;
    /*
        PSO PID 静态性能测试 执行
    */
    pso.init(true);
    pso.updateSwarm(true);


    // 适应度重置
    pso.gbest_fitness = 1e6;
    pso.pbest_fitness = 1e6;
    car_position = car_position_init;

    pso.get_best_param(pos_gbest, pos_zbest);    
    double static_best_P = pos_gbest[0];
    double static_best_I = pos_gbest[1];
    double static_best_D = pos_gbest[2];
    double static_best_FirstInPlaceTime_diff = performance_index.Best_Performance.FirstInPlaceTime_diff;
    double static_best_StableInPlaceTime_diff = performance_index.Best_Performance.StableInPlaceTime_diff;
    double static_best_max_overshoot = performance_index.Best_Performance.max_overshoot;

    /*
        PSO PID 动态性能测试 执行
    */
    pso.read_tube_pos();
    pso.init(false);
    pso.updateSwarm(false);


    // system("cat ../performance_param.txt");

    pso.get_best_param(pos_gbest, pos_zbest);

    printf("\n\n #### static_test pos_gbest KP = %f , KI = %f , KD = %f \n\n", static_best_P, static_best_I, static_best_D);
    printf("\n\n #### static_test Best_Performance FirstInPlaceTime_diff = %f , StableInPlaceTime_diff = %f , max_overshoot = %f \n\n",\
        static_best_FirstInPlaceTime_diff,
        static_best_StableInPlaceTime_diff,
        static_best_max_overshoot);
    
    printf("\n\n #### dynamic_test pos_gbest KP = %f , KI = %f , KD = %f \n\n", pos_gbest[0], pos_gbest[1], pos_gbest[2]);
    printf("\n\n #### dynamic_test gbest_fitness = %f \n\n", pso.gbest_fitness);
                

    // 绘制轨迹
    plt::figure_size(MATPLOT_WIDE, MATPLOT_HIGHT);
    plt::plot(static_cur_positions, {{"label", "aAxisRotCenterZ"}, {"color", "green"}, {"linewidth", "3.0"}});
    plt::plot(dynamic_cur_positions, {{"label", "aAxisRotCenterZ"}, {"color", "red"}, {"linewidth", "3.0"}});

    plt::plot(target_positions, {{"label", "tube_z_pos_data"}, {"color", "blue"}, {"linewidth", "3.0"}});
    plt::title("Car Position over Time");
    plt::xlabel("Time Step");
    plt::ylabel("Position");
    plt::grid(true);
    plt::show();

    return 0;
}
