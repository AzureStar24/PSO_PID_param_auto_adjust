#ifndef PSO_HPP
#define PSO_HPP

#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
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

#include "global_config.hpp"
#include "pid.hpp"

















class Servo_Performance {

private:

    //起始时间，初次到位时间，稳态时间（绝对）
    struct timeval init_time, FirstInPlaceTime, StableInPlaceTime;

    double move_dir_dis;        //移动距离，带方向
    int current_stable_count;


    enum direction {
        MOVE_DIRECTION_POSITIVE = 0,
        MOVE_DIRECTION_NEGATIVE,
    };

    direction move_direction;


public:
    bool FirstInPlace_flag = false;           //初次到位标志位
    bool stable_flag = false;     //稳态标志位
    int stable_flag_count = false;     //稳态标志位
    double dy_stable_in_place_err;



public:

    void Servo_Performance_init()
    {
        stable_flag_count = 0;
        stable_flag = false;
        dy_stable_in_place_err = ALLOW_STABLE_IN_PLACE_ERROR;
    }

    void Servo_Performance_calibration_init(double current_pos, double target_pos)
    {
        gettimeofday(&init_time, NULL);
        move_dir_dis = current_pos + target_pos;
        //stage_target_pos = target_pos - current_pos; //目标值 - 当前值 = 移动距离
        FirstInPlace_flag = false;
        stable_flag = false;
        current_stable_count = 0;
        Performance_output.max_overshoot = 1e6;
        Performance_output.StableInPlaceTime_diff = 1e6;
        Performance_output.FirstInPlaceTime_diff = 1e6;


        if(move_dir_dis > current_pos) {
            move_direction = MOVE_DIRECTION_POSITIVE;
        }else {
            move_direction = MOVE_DIRECTION_NEGATIVE;
        }
        return ;
    }

    void Servo_Performance_calibration_run_static(double current_pos)
    {

        if(stable_flag == true) {
            return ;
        }

        // 初次到位判断
        if(FirstInPlace_flag == false) {
            // 考虑有无超调，有超调时到不了允许误差，就用首次超过目标值作为初次到位时间
            // 无超调必然可以到允许误差
            if((move_direction == MOVE_DIRECTION_POSITIVE) && ((current_pos > target_pos)\
                         || (fabs(current_pos - target_pos) < ALLOW_FIRST_IN_PLACE_ERROR))) {
                gettimeofday(&FirstInPlaceTime, NULL);
                Performance_output.FirstInPlaceTime_diff = calculate_time_diff(FirstInPlaceTime, init_time);
                FirstInPlace_flag = true;
            }else if((move_direction == MOVE_DIRECTION_NEGATIVE) && ((current_pos < target_pos)\
                         || (fabs(current_pos - target_pos) < ALLOW_FIRST_IN_PLACE_ERROR))) {
                gettimeofday(&FirstInPlaceTime, NULL);
                Performance_output.FirstInPlaceTime_diff = calculate_time_diff(FirstInPlaceTime, init_time);
                FirstInPlace_flag = true;
            }
            // printf(" FirstInPlaceTime_diff current_pos = %f , move_direction = %d \n", current_pos, move_direction);
        }


        // 此处缺少无超调情况的处理
        // 如果做处理，那么无超调的情况下 max_overshoot = 0，适应度判断肯定更倾向于无超调的情况，也不确定无超调是不是最优解
        // 如果确定无超调一定是最优解，那就加判断，然后在 max_overshoot = 0 基础上再去判断其他适应度，寻找整体最优适应度


        //超调及稳态判断
        if(FirstInPlace_flag == true) {
            // 超调记录
            if(Performance_output.max_overshoot != 1e6) { //超调量初始值判断
                if(fabs(Performance_output.max_overshoot) < fabs(current_pos - target_pos)) {
                    Performance_output.max_overshoot = fabs(current_pos - target_pos);
                }
            }else {
                Performance_output.max_overshoot = fabs(current_pos - target_pos);
            }
            // printf("StableInPlaceTime_diff current_pos = %f , target_pos = %f , max_overshoot = %f\n", current_pos, target_pos, Performance_output.max_overshoot);

            // 稳态判断
#if USE_DYNAMIC_STABLE_ERROR
            if(fabs(current_pos - target_pos) <= dy_stable_in_place_err) {
#else
            if(fabs(current_pos - target_pos) <= ALLOW_STABLE_IN_PLACE_ERROR) {
#endif
                current_stable_count++;
                if(current_stable_count == STABLE_COUNT) {
                    gettimeofday(&StableInPlaceTime, NULL);
                    Performance_output.StableInPlaceTime_diff = calculate_time_diff(StableInPlaceTime, init_time);
                    printf(" STABLE_COUNT = %d , NOW is stable status , StableInPlaceTime_diff = %f\n", STABLE_COUNT, Performance_output.StableInPlaceTime_diff);                    
                    current_stable_count = 0;
                    stable_flag = true;
                }
            }else {
                current_stable_count = 0;
            }
        }

        return ;
    }

    void Servo_Performance_calibration_run_dynamic(double current_pos, int i)
    {
        if (i == 0) {
            dynamic_err[i] = 0.0;
        }else {
            dynamic_err[i] = current_pos - tube_pos_data[i-1];
            //printf(" i = %d , current_pos = %f , tube_pos_data = %f , dynamic_err = %f\n", i, current_pos, tube_pos_data[i-1], dynamic_err[i]);
        }

        return ;
    }



    double calculate_time_diff(struct timeval current_time, struct timeval before_time)
    {
        long long diff_sec, diff_usec;
        diff_sec = current_time.tv_sec - before_time.tv_sec;
        diff_usec = current_time.tv_usec - before_time.tv_usec;

        // printf("current_time : %ld.%06ld\n", current_time.tv_sec, current_time.tv_usec);
        // printf("before_time  : %ld.%06ld\n", current_time.tv_sec, current_time.tv_usec);
        // printf("diff_sec = %lld, diff_usec = %lld (us)\n", diff_sec, diff_usec);

        double diff_time = diff_usec * 1e-6 + diff_sec;
        return diff_time;
    }


    void Servo_Performance_calibration_end()
    {
    
        //printf(" max_overshoot = %.6f , FirstInPlaceTime_diff = %.6f , StableInPlaceTime_diff = %.6f\n", \
                        Performance_output.max_overshoot, Performance_output.FirstInPlaceTime_diff, Performance_output.StableInPlaceTime_diff);

        // char performance_param[512] = {0};
        // sprintf(performance_param, 
        //     "echo \"max_overshoot = %.6f , FirstInPlaceTime_diff = %.6f , StableInPlaceTime_diff = %.6f \" >> \"../performance_param.txt\"",
        //     Performance_output.max_overshoot, Performance_output.FirstInPlaceTime_diff, Performance_output.StableInPlaceTime_diff);

        // printf("performance_param = %s\n", performance_param);

        // system(performance_param);


        return ;
    }



    struct Performance_index{
        double FirstInPlaceTime_diff;       // 初次到位时间(上升时间)
        double StableInPlaceTime_diff;      //稳态时间差
        double max_overshoot;               //超调量
    };
    struct Performance_index Performance_output;
    struct Performance_index Best_Performance;

    Performance_index Servo_Performance_calibration_return()
    {
        return Performance_output;
    }

};


Servo_Performance performance_index;












































class AlgorithmPSO {
private:
    int Static_Dim = 3;                                // 参数的维度 (K_P, K_I, K_D)
    int Dynamic_Dim = 1;                                // 参数的维度 (K_P, K_I, K_D)
    int Dim = 0;                                // 参数的维度 (K_P, K_I, K_D)
    int SwarmSize = 40;                         // 粒子群大小
    vector<vector<double>> Swarm;     // 粒子的位置(二维数组)
    vector<vector<double>> VStep;     // 粒子的速度(二维数组)
    vector<double> Swarm_fitness;          // 粒子的适应度 (0.1*KP + 0.2*KI + 0.7*KD)
    vector<double> gbest;                  // 全局最优解
    vector<double> pbest;                  // 个人最优解

    double w = 0.6;                             // 惯性权重，经验值[0.4, 0.6, 0.9]
    double c1 = 2;                              // 个体认知因子
    double c2 = 2;                              // 社会认知因子
    // double Vmax[3] = {1, 0.1, 0.1};          // 速度上边界
    // double Vmin[3] = {-1, -0.1, -0.1};       // 速度下边界
    double Vmax[3] = {1, 1, 1};          // 速度上边界
    double Vmin[3] = {-1, -1, -1};       // 速度下边界
    double PosUb[3] = {10, 0.3, 0.3};          // 位置上边界
    double PosLb[3] = {0, 0, 0};               // 位置下边界
    double Weight[3] = {0.1, 0.2, 0.7};     // 适应度权重(超调占比最大，所以取小比例)
    
    double fitness = 1e6;

    

public:

    double gbest_fitness = 1e6;                 // 全局最优解的适应度
    double pbest_fitness = 1e6;                 // 个人最优解的适应度

public:

    double getMAX(double a, double b) {
        return a > b ? a : b;
    }

    double getMIN(double a, double b) {
        return a < b ? a : b;
    }

    // 初始化粒子位置、速度和适应度
    void init(bool is_static_test)
    {
        if(is_static_test) {
            Dim = Static_Dim;
        }else {
            Dim = Dynamic_Dim;
        }
        Swarm.resize(SwarmSize, vector<double>(Dim));
        VStep.resize(SwarmSize, vector<double>(Dim));
        Swarm_fitness.resize(SwarmSize);

        // 默认0，初始化为1e8，防止后续找最小值出问题
        for (int i = 0; i < (int)Swarm_fitness.size(); ++i) {
            Swarm_fitness[i] = 1e6;
        }

        // 初始化粒子位置
        for (int i = 0; i < SwarmSize; ++i) {
            for (int j = 0; j < Dim; ++j) {
                Swarm[i][j] = ((double)rand() / RAND_MAX) * (PosUb[j] - PosLb[j]) + PosLb[j]; // 随机生成粒子位置
                VStep[i][j] = ((double)rand() / RAND_MAX) * (Vmax[j] - Vmin[j]) + Vmin[j]; // 随机生成粒子速度
            }
        }

        // 计算每个粒子的适应度
        for (int i = 0; i < SwarmSize; ++i) {
            if(is_static_test) {
                Swarm_fitness[i] = evaluateFitness_static(Swarm[i], i);
            }else {
                Swarm_fitness[i] = evaluateFitness_dynamic(Swarm[i], i);
            }
            // 更新全局最优解
            if (Swarm_fitness[i] < gbest_fitness) {
                gbest = Swarm[i];
                gbest_fitness = Swarm_fitness[i];
                performance_index.Best_Performance = performance_index.Performance_output;

                printf(" FirstInPlaceTime_diff = %f , StableInPlaceTime_diff = %f , max_overshoot = %f\n",\
                            performance_index.Best_Performance.FirstInPlaceTime_diff,\
                            performance_index.Best_Performance.StableInPlaceTime_diff,\
                            performance_index.Best_Performance.max_overshoot);
                printf(" gbest_fitnessgbest_fitness = %f \n", gbest_fitness);
            
            }
            // 更新个体最优解
            if (Swarm_fitness[i] < pbest_fitness) {
                pbest = Swarm[i];
                pbest_fitness = Swarm_fitness[i];
            }
        }

        // for (int i = 0; i < (int)Swarm_fitness.size(); ++i) {
        //     printf(" Swarm_fitnessSwarm_fitness[%d] = %f \n", i, Swarm_fitness[i]);
        // }

        // // 初始化全局最优解，个人最优解
        // gbest_fitness = *min_element(Swarm_fitness.begin(), Swarm_fitness.end());
        // pbest_fitness = gbest_fitness;
        // int best_index = distance(Swarm_fitness.begin(), min_element(Swarm_fitness.begin(), Swarm_fitness.end()));
        // pbest = Swarm[best_index];
        // gbest = Swarm[best_index];
        // printf("init K_P = %f , K_I = %f , K_D = %f \n", pid_class.pid.K_P, pid_class.pid.K_I, pid_class.pid.K_D);
    }




    // 更新粒子群的位置和速度
    void updateSwarm(bool is_static_test)
    {
        //     for (int i = 0; i < tube_pos_data.size(); i++) {
        //         printf("tube_pos_data[%d] = %f\n" ,  i, tube_pos_data[i]);
        //     }
        if(is_static_test) {
            Dim = Static_Dim;
        }else {
            Dim = Dynamic_Dim;
        }
        for (int i = 0; i < SwarmSize; ++i) {
            for (int j = 0; j < Dim; ++j) {
                // 更新速度
                VStep[i][j] = w * VStep[i][j] + c1 * ((double)rand() / RAND_MAX) * (gbest[j] - Swarm[i][j])
                               + c2 * ((double)rand() / RAND_MAX) * (pbest[j] - Swarm[i][j]);
                // 限制速度
                if (VStep[i][j] > Vmax[j]) VStep[i][j] = Vmax[j];
                if (VStep[i][j] < Vmin[j]) VStep[i][j] = Vmin[j];

                // 更新位置
                Swarm[i][j] += VStep[i][j];
                if (Swarm[i][j] > PosUb[j]) Swarm[i][j] = PosUb[j];
                if (Swarm[i][j] < PosLb[j]) Swarm[i][j] = PosLb[j];
            }

            // 计算新适应度
            if(is_static_test) {
                Swarm_fitness[i] = evaluateFitness_static(Swarm[i], i);
            }else {
                Swarm_fitness[i] = evaluateFitness_dynamic(Swarm[i], i);
            }
            // 更新全局最优解
            if (Swarm_fitness[i] < gbest_fitness) {
                gbest = Swarm[i];
                gbest_fitness = Swarm_fitness[i];
                performance_index.Best_Performance = performance_index.Performance_output;
            }
            // 更新个体最优解
            if (Swarm_fitness[i] < pbest_fitness) {
                pbest = Swarm[i];
                pbest_fitness = Swarm_fitness[i];
            }

            // for (int j = 0; j < Dim; ++j) {
            //     printf("updateSwarm i = %d , VStep = %f , Swarm = %f , Swarm_fitness = %f , gbest = %f , pbest = %f , gbest_fitness = %f , pbest_fitness = %f \n", \
            //     i, VStep[i][j], Swarm[i][j], Swarm_fitness[j], gbest[j], pbest[j], gbest_fitness, pbest_fitness);
            // }
        }
    }



    //适应度函数（静态）
    double evaluateFitness_static(const vector<double>& params, int counti)
    {
        car_position = car_position_init;
        static_cur_positions.push_back(car_position);
        performance_index.Servo_Performance_calibration_init(car_position, target_pos);

        pid_class.pid.K_P = params[0];
        pid_class.pid.K_I = params[1];
        pid_class.pid.K_D = params[2];
        printf("update K_P = %f , K_I = %f , K_D = %f \n", pid_class.pid.K_P, pid_class.pid.K_I, pid_class.pid.K_D);

        for (int i = 0; i < steps; ++i)
        {
            if(performance_index.stable_flag == true) {
                break;
            }
            /*
                控制方法
            */
            // //自适应PID
            // control_input = adaptive_pid.control(error, dt, noise);

            // 位置PID
            // control_input = position_control(error, dt);
            
            // 增量PID
            double control_input = pid_class.increment_control(target_pos - car_position, dt);


            #if GAUSSIAN_FLAG
                /*
                    高斯噪声
                */
                if(i % 20 == 0) {
                    use_gaussian_noise = true;
                    printf("use_gaussian_noise = true\n");
                }else {
                    use_gaussian_noise = false;
                    printf("use_gaussian_noise = false\n");
                }
                noise = use_gaussian_noise ? generateGaussianNoise(0.0, 4) : 0.0;
                car_position += control_input * dt + noise;
            #else
                // 更新位置
                car_position += control_input * dt;
            #endif

            // 性能测试
            performance_index.Servo_Performance_calibration_run_static(car_position);
            //printf(" car_positioncar_positioncar_positioncar_position = %f \n", car_position);

            // 记录位置, 时间
            static_cur_positions.push_back(car_position);
            times.push_back(current_time);

            // 更新时间
            current_time += dt;
        }

#if USE_DYNAMIC_STABLE_ERROR
        // 解决一直无法到稳态的情况
        if(performance_index.stable_flag == false) {
            performance_index.stable_flag_count++;
            if(performance_index.stable_flag_count >= 20) {
                //两种动态误差方法
                // performance_index.dy_stable_in_place_err = performance_index.single_final_err + ALLOW_STABLE_IN_PLACE_ERROR;
                performance_index.dy_stable_in_place_err += ALLOW_STABLE_IN_PLACE_ERROR;
                performance_index.stable_flag_count = 0;
                printf("task_follow_pso stable_flag_count >= 10 , increase dy_stable_in_place_err = %f\n", performance_index.dy_stable_in_place_err);
            }
        }else {
            performance_index.stable_flag_count = 0;
        }
#else
#endif


        // 性能测试结果输出
        performance_index.Servo_Performance_calibration_end();

        //适应度计算
        double fitness = Weight[0] * performance_index.Performance_output.FirstInPlaceTime_diff \
                        + Weight[1] * performance_index.Performance_output.StableInPlaceTime_diff \
                        + Weight[2] * performance_index.Performance_output.max_overshoot;

        printf(" evaluateFitness Weight[0] = %f , Weight[1] = %f , Weight[2] = %f , FirstInPlaceTime_diff = %f , StableInPlaceTime_diff = %f , max_overshoot = %f\n", \
                    Weight[0], Weight[1], Weight[2], performance_index.Performance_output.FirstInPlaceTime_diff, \
                    performance_index.Performance_output.StableInPlaceTime_diff, performance_index.Performance_output.max_overshoot);
        printf("evaluateFitness_static counti = %d , fitness fitness fitness fitness = %f \n\n", counti, fitness);
        // 目标是最小化超调、上升时间和稳态时间
        return  fitness;
    }



    //适应度函数（动态）
    double evaluateFitness_dynamic(const vector<double>& params, int counti)
    {

        //car_position = car_position_init;
        performance_index.Servo_Performance_calibration_init(car_position, target_pos);
        fitness = 0.0;

        pid_class.pid.K_P = params[0];
        pid_class.pid.K_I = params[1];
        pid_class.pid.K_D = params[2];
        printf("update K_P = %f , K_I = %f , K_D = %f \n", pid_class.pid.K_P, pid_class.pid.K_I, pid_class.pid.K_D);

        for (int i = 0; i < tube_pos_data.size(); ++i)
        {
            /*
                控制方法
            */
            // //自适应PID
            // control_input = adaptive_pid.control(error, dt, noise);

            // 位置PID
            // control_input = position_control(error, dt);
            
            // 增量PID
            double control_input = pid_class.increment_control(tube_pos_data[i] - car_position, dt);

            #if GAUSSIAN_FLAG
                /*
                    高斯噪声
                */
                if(i % 20 == 0) {
                    use_gaussian_noise = true;
                    printf("use_gaussian_noise = true\n");
                }else {
                    use_gaussian_noise = false;
                    printf("use_gaussian_noise = false\n");
                }
                noise = use_gaussian_noise ? generateGaussianNoise(0.0, 4) : 0.0;
                car_position += control_input * dt + noise;
            #else
                // 更新位置
                car_position += control_input * dt;
            #endif

            // 性能测试
            performance_index.Servo_Performance_calibration_run_dynamic(car_position, i);
            //printf(" car_positioncar_positioncar_positioncar_position = %f \n", car_position);

            // 记录位置, 时间
            dynamic_cur_positions.push_back(car_position - 20);
            target_positions.push_back(tube_pos_data[i] - 20);
            times.push_back(current_time);

            // 更新时间
            current_time += dt;
        }

        
        for(int i = 0; i < dynamic_err.size(); i++) {
            fitness += fabs(dynamic_err[i]);
        }

        printf("evaluateFitness_dynamic counti = %d , fitness fitness fitness fitness = %f \n\n", counti, fitness); 

        // 目标是最小化超调、上升时间和稳态时间
        return  fitness;
    }





    void get_best_param(vector<double>& pos_gbest, vector<double> & pos_pbest)
    {
        pos_gbest = gbest;
        pos_pbest = pbest;
        
        return ;
    }
    
     



    int fread_file_as_doubles(string file_name, vector<double> &output_array, int max_lines) {
        ifstream file(file_name); // 打开文件
        if (!file.is_open()) {
            cerr << "Error: Unable to open file: " << file_name << endl;
            return -1;
        }

        string line_data;
        int count = 0; // 已读取行计数

        while (getline(file, line_data)) {
            // 将读取的行转换为 double 类型
            try {
                double value = strtod(line_data.c_str(), nullptr); // 转换为 double
                output_array.push_back(value); // 存储到 vector 中
                count++;
            } catch (const exception &e) {
                cerr << "Error: Failed to parse line " << count + 1 << ": " << line_data << endl;
                continue; // 跳过无法解析的行
            }

            // 如果超过最大行数，停止读取
            if (count >= max_lines) {
                break;
            }
        }

        if (file.bad()) {
            cerr << "Error: Failed to read from file: " << file_name << endl;
            return -1;
        }

        file.close();
        return count; // 返回成功读取的行数
    }

    int read_tube_pos() {
        string filename = "../tube_corner_pos.txt";
        int pso_count = 1000; // 最大读取行数
        vector<double> data;
        tube_pos_data.resize(pso_count);

        int lines_read = fread_file_as_doubles(filename, data, pso_count);
        if (lines_read == -1) {
            return -1; // 出错，退出
        }

        tube_pos_data = data;

        // cout << "成功读取了 " << data.size() << " 行数据：" << endl;
        // for (int i = 0; i < data.size(); i++) {
        //     printf("data[%d] = %f\n", i, data[i]);
        // }
        // for (int i = 0; i < tube_pos_data.size(); i++) {
        //     printf(" tube_pos_data[%d] = %f\n", i, tube_pos_data[i]);
        // }

        return 0; // 正常退出
    }



    // int fread_file_as_doubles(char *file_name, double *output_array, int max_lines)
    // {
    //     FILE *file = fopen(file_name, "r"); // 打开文件，"r" 模式表示只读
    //     if (file == NULL)
    //     {
    //         perror("Error opening file for reading");
    //         return -1;
    //     }

    //     char line[128]; // 用于存储每一行的数据
    //     int count = 0;

    //     while (fgets(line, sizeof(line), file) != NULL)
    //     {
    //         //printf(" fgets(line, sizeof(line), file) = %d \n", fgets(line, sizeof(line), file));
    //         // 将读取的行转换为 double 类型并存储到数组中
    //         output_array[count] = strtod(line, NULL);
    //         count++;

    //         // 如果超过最大行数，停止读取
    //         if (count >= max_lines)
    //         {
    //             break;
    //         }
    //     }

    //     if (ferror(file))
    //     {
    //         perror("Error reading from file");
    //         fclose(file);
    //         return -1;
    //     }

    //     fclose(file);
    //     return count; // 返回成功读取的行数
    // }



    // int read_tube_pos() {
    //     char filename[128] = "/home/zqj/1git_code/code/algo/control_algo_cpp/PID_auto_adjust/tube_corner_pos.txt";
    //     const int pso_count = 1000; // 最大读取行数
    //     double tube_pos_data11[pso_count] = {0.0};

    //     int lines_read = fread_file_as_doubles(filename, tube_pos_data11, pso_count);
    //     if (lines_read == -1) {
    //         return -1; // 出错，退出
    //     }

    //     cout << "成功读取了 " << lines_read << " 行数据：" << endl;
    //     for (int i = 0; i < lines_read; i++) {
    //         cout << "data[" << i << "] = " << tube_pos_data11[i] << endl;
    //     }

    //     return 0; // 正常退出
    // }



};




AlgorithmPSO pso;



#endif
