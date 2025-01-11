# PSO_PID_param_auto_adjust

## Build
```shell
# 依赖
sudo apt-get install python-matplotlib python-numpy python2.7-dev

# 编译
cd source
./1build_all.sh
```


## PSO-PID自整定流程
参考：MATLAB智能算法30个案例 

![](./picture/22.png)


## PSO公式
 
**一、粒子的速度更新公式为：** 

$$v_{i,j}(t) + c_1 \cdot r_1 \cdot \left( p_{i,j}(t) - x_{i,j}(t) \right) + c_2 \cdot r_2 \cdot \left( g_j(t) - x_{i,j}(t) \right)$$

- $v_{i,j}(t)$：粒子 i 在第 t 代的第 j 维速度；
- $w$：惯性权重，控制粒子当前速度的影响；
- $c_1, c_2$：学习因子或加速常数，通常取值在 [0, 2] 之间；
- $r_1, r_2$：随机数，均匀分布在 [0, 1] 区间；
- $p_{i,j}(t)$：粒子 i 在第 j 维的历史最优位置（个体最优）；
- $g_j(t)$：种群在第 j 维的全局最优位置（全局最优）；
- $x_{i,j}(t)$：粒子 i 在第 t 代的第 j 维位置。


**二、粒子的位置根据速度更新：** 
 
$$x_{i,j}(t+1) = x_{i,j}(t) + v_{i,j}(t+1)$$

- $x_{i,j}(t)$：粒子 i 在第 t 代的第 j 维位置；
- $v_{i,j}(t+1)$：更新后的速度。


**三、限制速度范围：** 

$$v_{i,j}(t) \in \left[ -v_{\text{max}}, v_{\text{max}} \right]$$



**四、惯性权重的调整：** 

惯性权重 w 的值可以是固定的（代码中使用固定权重），也可以随着迭代逐渐衰减，例如线性递减： 

$$w = w_{\text{max}} - \frac{w_{\text{max}} - w_{\text{min}}}{\text{iter}_{\text{max}}} \cdot t$$

- $w_{\text{max}}, w_{\text{min}}$：惯性权重的最大值和最小值；
- $\text{iter}_{\text{max}}$：最大迭代次数；
- $t$：当前迭代次数。



## 两种适应度函数
静态性能测试：步进固定距离 

静态适应度输出：上升时间（初次到位时间），稳态时间，超调量

 
动态性能测试：跟随指定轨迹移动 

动态适应度输出：当前时刻 t 位置和 t-1 时刻目标位置的误差 

![](./picture/33.png)

