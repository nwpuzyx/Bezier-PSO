%% 复现《基于Bezier和改进PSO算法的风环境下翼伞航迹规划---Daniel---2019.12.34 %% 
clc;
clear;

%% 初始参数
X0 = [0 0];   XF = [64.3 0];   % 初始位置和终点位置
Dir_0 = 0;    Dir_f = 0;     % 初始航向角和终点航向角

%% 建立风环境模型
load wind                     
x2=x(:,:,5); y2=y(:,:,5);
u2=u(:,:,5); v2=v(:,:,5);                 % 导入数据
% figure()                                
Q=quiver(x2-70,y2-37.5,u2,v2);           % 绘图，从原点开始
Q.AutoScaleFactor=1.2;                    % 矢量图箭头参数
x_max = max(max(x2-70));
y_max = max(max(y2-37.5));
u_max = max(max(u2));
v_max = max(max(v2));

[m, n] = size(x2);                        % 数据大小

line([0,x_max],[0,0]);                    % 绘制起始点与终点的连线ST

Zero_Point = [];                          % ST线上的点数据
for i =1:n*m
    Input = [x2(i) y2(i) u2(i) v2(i)];
    if abs(y2(i)-37.5)<0.1
        Zero_Point = [Zero_Point; Input];
    end
end
Zero_Point = sortrows(Zero_Point,1);      % 按照升序排列
[m_z, n_z]= size(Zero_Point);             % ST线上点的大小

% 计算 N 和过零点   N表示预定轨迹中Beizer曲线条数
N_zero_point = 0;                             
index = [];
for i =1:m_z-1
    Point_i = Zero_Point(i, :);
    v0 = Point_i(4);
    Point_i_ = Zero_Point(i+1, :);
    v1 = Point_i_(4);
    if v0*v1<0
        N_zero_point = N_zero_point + 1;
        index = [index i];
    end
end
disp(['预定轨迹中Bezier曲线条数',num2str(N_zero_point)]);
disp(['过零点个数为',num2str(N_zero_point),'   分别为第',num2str(index(1)),'和',num2str(index(2)),'个点']);
for i =1:N_zero_point
    Point_i = Zero_Point(index(i), :);
    if Point_i(4)<0
        disp(['第',num2str(i),'个过零点左侧为负，右侧为正']);
    end
    if Point_i(4)>0
        disp(['第',num2str(i),'个过零点左侧为正，右侧为负']);
    end
end
disp(['第一个过零点坐标为 ',num2str(Zero_Point(index(1),:))]);
disp(['第二个过零点坐标为 ',num2str(Zero_Point(index(2),:))]);

% 计算过零点的边界值
Zero_cross_1 = Zero_Point(index(1),:);
Zero_cross_2 = Zero_Point(index(2),:);

% 找出与过零点横坐标相等的点，查找另一边界值
N_1_Point = [];
N_2_Point = [];
for i =1:n*m
    Input = [x2(i) y2(i) u2(i) v2(i)];
    if abs(Input(1)-Zero_cross_1(1))<0.1
        N_1_Point = [N_1_Point; Input];
    end
    if abs(Input(1)-Zero_cross_2(1))<0.1
        N_2_Point = [N_2_Point; Input];
    end
end
[m_y_1, n_y_1] = size(N_1_Point);
[m_y_2, n_y_2] = size(N_2_Point);
N_1_Point_sort = [];
N_2_Point_sort = [];
Zero_Point_y_1 = Zero_Point(index(1),:);
Zero_Point_y_2 = Zero_Point(index(2),:);

for i = 1:m_y_1
    Point_i = N_1_Point(i,:);
    if Point_i(2) <= Zero_Point_y_1(2)
        N_1_Point_sort = [N_1_Point_sort; Point_i];
    end
end
for i = 1:m_y_2
    Point_i = N_2_Point(i,:);
    if Point_i(2) >= Zero_Point_y_2(2)
        N_2_Point_sort = [N_2_Point_sort; Point_i];
    end
end
N_1_Point_sort = sortrows(N_1_Point_sort,2, 'descend');
N_2_Point_sort = sortrows(N_2_Point_sort,2);                       % 需要的仅位于一侧的点

Boundary_1 = [];                                        
Boundary_2 = [];                                                   % 初始化除过零点之外的另两个边界值

for i=1:length(N_1_Point_sort)
    Point_i = N_1_Point_sort(i,:);
    if abs(Point_i(3))<10                                          % 当风速的水平分量小于某较小值时，取为另一边界值
        Boundary_1 = [Boundary_1; Point_i];
        break
    end
end

for i=1:length(N_2_Point_sort)
    Point_i = N_2_Point_sort(i,:);
    if abs(Point_i(3))<10
        Boundary_2 = [Boundary_2; Point_i];
        break
    end
    Boundary_2 = Point_i;
end

Splice_point_1 = Zero_Point(index(1),:);
Splice_point_2 = Boundary_1;
Splice_point_3 = Zero_Point(index(2),:);
Splice_point_4 = Boundary_2;

disp(['拼接点边界坐标为 ',num2str(Splice_point_1)]);
disp(['拼接点边界坐标为 ',num2str(Splice_point_2)]);
disp(['拼接点边界坐标为 ',num2str(Splice_point_3)]);
disp(['拼接点边界坐标为 ',num2str(Splice_point_4)]);                  % 输出四个边界坐标（包含两个过零点）

%% Bezier： 绘制 Bezier 曲线

% 测试产生Beizer曲线的算法
x = [0 1 1 2];
y = [0 0 -1 -1];
t = 0:0.01:1;
xx_0 = 0; yy_0 = 0;
b0 = (1-t).^3;
b1 = 3 * t .* (1-t) .^2;
b2 = 3 * t .^2 .*(1-t);
b3 = t.^3;
xx = xx_0 + b0*x(1) + b1*x(2) + b2*x(3) + b3*x(4);
yy = yy_0 + b0*y(1) + b1*y(2) + b2*y(3) + b3*y(4); 

x_ = [2 3 1 4];
y_ = [1 1 -2 -2];
xx_ = xx_0 + b0*x_(1) + b1*x_(2) + b2*x_(3) + b3*x_(4);
yy_ = yy_0 + b0*y_(1) + b1*y_(2) + b2*y_(3) + b3*y_(4); 

x__ = [4 6 7 8];
y__ = [-2 -2 0 0];
xx__ = xx_0 + b0*x__(1) + b1*x__(2) + b2*x__(3) + b3*x__(4);
yy__ = yy_0 + b0*y__(1) + b1*y__(2) + b2*y__(3) + b3*y__(4); 

figure()
hold on 
plot(xx, yy);
plot(x, y);
plot(xx_, yy_);
plot(x_, y_);
plot(xx__, yy__);
plot(x__, y__);
for i=1:4
    plot(x(i), y(i), 'o');
    plot(x_(i), y_(i), 'o');
    plot(x__(i), y__(i), 'o');
end
hold off

%% PSO: 粒子群算法

% 初始化种群
[m, n]=size(x2);                                              % 离散点个数

N = 20;                                                       % 初始种群个数
d = 2;                                                        % 空间维数
ger = 500;                                                    % 最大迭代次数     
limit_1 = [Splice_point_2(2), Splice_point_1(2)];            
limit_2 = [Splice_point_3(2), Splice_point_4(2)];             % 设置拼接点的位置参数限制
limit_3 = [x_max, y_max];                                     % 设置其余点的位置参数约束
vlimit = [-1, 1];                                             % 设置速度限制
w = 0.8;                                                      % 惯性权重
c1 = 0.5;                                                     % 自我学习因子
c2 = 0.5;                                                     % 群体学习因子 
acc = 1;                                                      % 轨迹精度/步长
x = [];                                                       % 初始粒子集

for i = 1:N                                                                            % 初始种群的位置    
    x = [x; [Splice_point_1(1), limit_1(1)+(limit_1(2)-limit_1(1))*rand(1),...         % 拼接点1
             Splice_point_3(1), limit_2(1)+(limit_2(2)-limit_2(1))*rand(1),...         % 拼接点2
             rand(1)*x_max, 0,...                                                      % 起点右侧点
             rand(1)*x_max, 0]];                                                       % 终点左侧点
end

% 根据初始粒子的位置获取最近点的速度值
X_v = [];
for jj = 1:N
    input_x = x(jj,:);
    particle_ = particle(input_x);
    X_v = [X_v; particle_];
end
rand_point = [];                             % 拼接点左右两侧的四个任意点
for i =1 :N
    X_splice_point_1 = X_v(i,1:4);
    X_splice_point_2 = X_v(i,5:8);
    
    Slope_1 = X_splice_point_1(4)/X_splice_point_1(3);
    Slope_2 = X_splice_point_2(4)/X_splice_point_2(3);
    
    rand_x_1_r = rand(1)*100 + X_splice_point_1(1);
    rand_y_1_r = Slope_1*(rand_x_1_r - X_splice_point_1(1)) + X_splice_point_1(2);         % 第一个拼接点右侧的任意点
    rand_x_1_l = -rand(1)*100 + X_splice_point_1(1);
    rand_y_1_l = Slope_1*(rand_x_1_l - X_splice_point_1(1)) + X_splice_point_1(2);         % 第一个拼接点左侧的任意点
    
    rand_x_2_r = rand(1)*100 + X_splice_point_2(1);
    rand_y_2_r = Slope_2*(rand_x_2_r - X_splice_point_2(1)) + X_splice_point_2(2);         % 第二个拼接点右侧的任意点
    rand_x_2_l = -rand(1)*100 + X_splice_point_2(1);
    rand_y_2_l = -Slope_2*(rand_x_2_l - X_splice_point_2(1)) + X_splice_point_2(2);        % 第二个拼接点左侧的任意点
    
    rand_point = [rand_point;...
                  rand_x_1_l, rand_y_1_l, rand_x_1_r, rand_y_1_r,...
                  rand_x_2_l, rand_y_2_l, rand_x_2_r, rand_y_2_r];
end
X_ALL = [X_v rand_point];

v = rand(N, d);                  % 初始种群的速度 
xm = x;                          % 每个个体的历史最佳位置
ym = zeros(1, d);                % 种群的历史最佳位置
fxm = zeros(N, 1);               % 每个个体的历史最佳适应度
fym = -inf;                      % 种群历史最佳适应度

% 计算初始化粒子的位置、速度
index_N_1 = [];
index_N_2 = [];
for i=1:N
    x_i = x(i,:);
    distance_min = abs(x_i(2)-N_1_Point_sort(1,2));
    index_N_ii = 1;
    for ii=1:length(N_1_Point_sort)
        Point_sort_ii = N_1_Point_sort(ii,:);
        distance_ii = abs(x_i(2)-(Point_sort_ii(2)));
        if distance_ii < distance_min
            distance_min = distance_ii;
            index_N_ii = ii;
        end
    end
    
    distance_min = abs(x_i(2)-N_2_Point_sort(1,2));
    index_N_jj = 1;
    for jj=1:length(N_2_Point_sort)
        Point_sort_jj = N_2_Point_sort(jj,:);  
        distance_jj = abs(x_i(4)-(Point_sort_jj(2)));
        if distance_jj < distance_min
            distance_min = distance_jj;
            index_N_jj = jj;
        end
    end
    index_N_1 = [index_N_1 index_N_ii];
    index_N_2 = [index_N_2 index_N_jj];
end

x_uv =[];
for i=1:length(index_N_1)
    x_uv = [x_uv; 
            x(i,1), x(i,2), N_1_Point_sort(index_N_1(i),3), N_1_Point_sort(index_N_1(i),4),...
            x(i,3), x(i,4), N_2_Point_sort(index_N_2(i),3), N_2_Point_sort(index_N_2(i),4)];
end

figure(3)
hold on 
plot(xm(:,1)-70, xm(:,2)-37.5, 'ro');
plot(xm(:,3)-70, xm(:,4)-37.5, 'ro');
title('初始粒子');
hold off

% 初始粒子状态显示
figure()
hold on 

for i = 1:N
    X_use = X_ALL(i,:);
    
    t = 0:0.01:1;
    xx = 0; yy = 0;
    b0 = (1-t).^3;
    b1 = 3 * t .* (1-t) .^2;
    b2 = 3 * t .^2 .*(1-t);
    b3 = t.^3;
    
    x = [0 X_use(9) X_use(17)-70 X_use(1)-70];
    y = [0 X_use(10) X_use(18)-37.5 X_use(2)-37.5];
    xx = xx + b0*x(1) + b1*x(2) + b2*x(3) + b3*x(4);
    yy = yy + b0*y(1) + b1*y(2) + b2*y(3) + b3*y(4); 
    
    xx_ = 0; yy_ = 0;
    x_ = [X_use(1)-70 X_use(19)-70 X_use(21)-70 X_use(5)-70];
    y_ = [X_use(2)-37.5 X_use(20)-37.5 X_use(22)-37.5 X_use(6)-37.5];
    xx_ = xx_ + b0*x_(1) + b1*x_(2) + b2*x_(3) + b3*x_(4);
    yy_ = yy_ + b0*y_(1) + b1*y_(2) + b2*y_(3) + b3*y_(4); 
    
    xx__ = 0; yy__ = 0;
    x__ = [X_use(5)-70 X_use(23)-70 X_use(13) 64.3];
    y__ = [X_use(6)-37.5 X_use(24)-37.5 X_use(14) 0];
    xx__ = xx__ + b0*x__(1) + b1*x__(2) + b2*x__(3) + b3*x__(4);
    yy__ = yy__+ b0*y__(1) + b1*y__(2) + b2*y__(3) + b3*y__(4); 
    
    plot(xx, yy);
    % plot(x, y);
    plot(xx_, yy_);
    % plot(x_, y_);
    plot(xx__, yy__);
    % plot(x__, y__);
    for i=1:4
        plot(x(i), y(i), 'o');
        plot(x_(i), y_(i), 'o');
        plot(x__(i), y__(i), 'o');
    end
end
hold off

X_Input = [];
for i = 1:101
    X_Input = [X_Input; xx(i), yy(i); xx_(i), yy_(i); xx__(i), yy__(i)];
end

[size_input_m, size_input_n] = size(X_Input);
distance_min = sqrt(Input(1)^2 + Input(2)^2);
particle_v =[];
for i=1:size_input_m
    Input = X_Input(i,:);
    for i=1:m*n
        Point_i = [x2(i) y2(i) u2(i) v2(i)];
        distance = sqrt((Point_i(1)-Input(1))^2 + (Point_i(2)-Input(2))^2);
        if distance < distance_min
            distance_min = distance;
            index = i;
            U = [Point_i(3), Point_i(4)];
        end
    end
    Output = [Input(1), Input(2), U(1), U(2)];
    particle_v = [particle_v; Output];
end

[size_m, size_n] = size(X_ALL);
X = [];                                % 获得仅含x和y的数据集
for i = 1:size_m
    X_I = X_ALL(i,:);
    X = [X; X_I(1), X_I(2), X_I(5), X_I(6), X_I(9), X_I(10), X_I(13), X_I(14)...
         X_I(17), X_I(18), X_I(19), X_I(20), X_I(21), X_I(22), X_I(23), X_I(24)];
end
X_RAW = X;
v = rand(N, 16);                       % 初始种群的速度
xm = X;                                % 每个个体的历史最佳位置
ym = zeros(1, d);                      % 种群的历史最佳位置
fxm = zeros(N, 1);                     % 每个个体的历史最佳适应度
fym = -inf;                            % 种群历史最佳适应度

%% 适应度函数
% 群体更新
% 边界位置处理
limit_low = [X_RAW(1,1), limit_1(1), X_RAW(1,3), limit_2(1), -inf,  0, 0,     0, -inf,       -inf, X_RAW(1,1), -inf, -inf,       -inf, X_RAW(1,3), -inf];
limit_upp = [X_RAW(1,1), limit_1(2), X_RAW(1,3), limit_2(2), x_max, 0, x_max, 0, X_RAW(1,1), inf,  inf,        inf,  X_RAW(1,3), inf,  inf,        inf];
iter = 1;
record = zeros(ger, 1);          % 记录器
while iter <= ger
     fx = [];
     for iii = 1:N
         fitness = fitness_self(X(iii,:));                % 个体当前适应度   
         fx = [fx fitness];                               % 总的个体适应度
     end
     for i = 1:N
        if fxm(i) < fx(i)
            fxm(i) = fx(i);                               % 更新个体历史最佳适应度 
            xm(i,:) = X(i,:);                             % 更新个体历史最佳位置
        end 
     end

    if fym < max(fxm)
            [fym, nmax] = max(fxm);                        % 更新群体历史最佳适应度
            ym = xm(nmax, :);                              % 更新群体历史最佳位置
    end

    v = v * w + c1 * rand * (xm - X) + c2 * rand * (repmat(ym, N, 1) - X);   % 速度更新
    
    % 边界速度处理
    v(v > vlimit(2)) = vlimit(2);
    v(v < vlimit(1)) = vlimit(1);
    
    % 位置更新
    X = X + v;
    
    % 保证拼接点和左右两点在一条线上
    for ii = 1:N
        x_3 = X(ii, 9); y_3 = X(ii, 10); 
        x_4 = X(ii, 1); y_4 = X(ii, 2);
        x_5 = X(ii, 11); y_5 = X(ii, 12);
        X(ii, 12) = y_4 + ((y_4-y_3)/(x_4-x_3))*(x_5 - x_4);
        
        x_6 = X(ii, 13); y_6 = X(ii, 14); 
        x_7 = X(ii, 3); y_7 = X(ii, 4);
        x_8 = X(ii, 15); y_8 = X(ii, 16);
        X(ii, 12) = y_7 + ((y_7-y_6)/(x_7-x_6))*(x_8 - x_7);
    end
    
    % 位置限
    for ii = 1:N        
        for i = 1:length(limit_low)
            if X(ii, i) > limit_upp(i)
                X(ii, i) = limit_upp(i);
            end
            if X(ii, i) < limit_low(i)
                X(ii, i) = limit_low(i);
            end
        end
    end
    
    record(iter) = fym;                                       % 最大值记录
%   x0 = 0 : 0.01 : 20;
%   plot(x0, f(x0), 'b-', x, f(x), 'ro');title('状态位置变化')
%   pause(0.1)
    iter = iter+1;
end

disp(['最大值：',num2str(fym)]);
disp(['变量取值：',num2str(ym)]);

t = 0:0.01:1;
xx = 0; yy = 0;
b0 = (1-t).^3;
b1 = 3 * t .* (1-t) .^2;
b2 = 3 * t .^2 .*(1-t);
b3 = t.^3;
    
x = [0 ym(5) ym(9)-70 ym(1)-70];
y = [0 ym(6) ym(10)-37.5 ym(2)-37.5];
xx = xx + b0*x(1) + b1*x(2) + b2*x(3) + b3*x(4);
yy = yy + b0*y(1) + b1*y(2) + b2*y(3) + b3*y(4); 
    
xx_ = 0; yy_ = 0;
x_ = [ym(1)-70 ym(11)-70 ym(13)-70 ym(3)-70];
y_ = [ym(2)-37.5 ym(12)-37.5 ym(14)-37.5 ym(4)-37.5];
xx_ = xx_ + b0*x_(1) + b1*x_(2) + b2*x_(3) + b3*x_(4);
yy_ = yy_ + b0*y_(1) + b1*y_(2) + b2*y_(3) + b3*y_(4); 
    
xx__ = 0; yy__ = 0;
x__ = [X_use(3)-70 X_use(15)-70 X_use(7) 64.3];
y__ = [X_use(4)-37.5 X_use(16)-37.5 X_use(8) 0];
xx__ = xx__ + b0*x__(1) + b1*x__(2) + b2*x__(3) + b3*x__(4);
yy__ = yy__+ b0*y__(1) + b1*y__(2) + b2*y__(3) + b3*y__(4); 
    
plot(xx, yy);
% plot(x, y);
plot(xx_, yy_);
% plot(x_, y_);
plot(xx__, yy__);
% plot(x__, y__);
for i=1:4
    plot(x(i), y(i), 'o');
    plot(x_(i), y_(i), 'o');
    plot(x__(i), y__(i), 'o');
end

hold off