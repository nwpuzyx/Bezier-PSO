###---Daniel---2020.01.01---###
本代码是复现论文“基于Beizer和改进PSO算法的风环境下翼伞航迹规划”的代码，不同之处在于此代码的代价函数选用的是
航迹长度和角度两个指标，期望航迹长度尽可能地短，航迹角度尽可能地与风速方向相同。

平台：MATLAB2018b，系统：WIN10

*main.m                 主函数
*particle.m              计算最邻近的风矢量的速度值，并将其作为该点的速度值
*fitness_self.m         计算每一个样本点的适应度值   

English:
This code is the code that reproduces the paper 
"Planning of Parachute in Wind Environment Based on Beizer and Improved PSO Algorithm". 
The difference is that the cost function of this code uses track length and angle are two indicators. 
It is expected that the track length is as short as possible and the track angle is as same as the wind speed direction.

Platform: MATLAB2018b， System:WIN10

*main.m                 main function
*particle.m             get the speed vector of the nearest wind point, as the speed of the track point
*fitness_self.m        get the fitness function of every track point