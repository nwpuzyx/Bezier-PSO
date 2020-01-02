%% ���֡�����Bezier�͸Ľ�PSO�㷨�ķ绷������ɡ�����滮---Daniel---2019.12.34 %% 
clc;
clear;

%% ��ʼ����
X0 = [0 0];   XF = [64.3 0];   % ��ʼλ�ú��յ�λ��
Dir_0 = 0;    Dir_f = 0;     % ��ʼ����Ǻ��յ㺽���

%% �����绷��ģ��
load wind                     
x2=x(:,:,5); y2=y(:,:,5);
u2=u(:,:,5); v2=v(:,:,5);                 % ��������
% figure()                                
Q=quiver(x2-70,y2-37.5,u2,v2);           % ��ͼ����ԭ�㿪ʼ
Q.AutoScaleFactor=1.2;                    % ʸ��ͼ��ͷ����
x_max = max(max(x2-70));
y_max = max(max(y2-37.5));
u_max = max(max(u2));
v_max = max(max(v2));

[m, n] = size(x2);                        % ���ݴ�С

line([0,x_max],[0,0]);                    % ������ʼ�����յ������ST

Zero_Point = [];                          % ST���ϵĵ�����
for i =1:n*m
    Input = [x2(i) y2(i) u2(i) v2(i)];
    if abs(y2(i)-37.5)<0.1
        Zero_Point = [Zero_Point; Input];
    end
end
Zero_Point = sortrows(Zero_Point,1);      % ������������
[m_z, n_z]= size(Zero_Point);             % ST���ϵ�Ĵ�С

% ���� N �͹����   N��ʾԤ���켣��Beizer��������
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
disp(['Ԥ���켣��Bezier��������',num2str(N_zero_point)]);
disp(['��������Ϊ',num2str(N_zero_point),'   �ֱ�Ϊ��',num2str(index(1)),'��',num2str(index(2)),'����']);
for i =1:N_zero_point
    Point_i = Zero_Point(index(i), :);
    if Point_i(4)<0
        disp(['��',num2str(i),'����������Ϊ�����Ҳ�Ϊ��']);
    end
    if Point_i(4)>0
        disp(['��',num2str(i),'����������Ϊ�����Ҳ�Ϊ��']);
    end
end
disp(['��һ�����������Ϊ ',num2str(Zero_Point(index(1),:))]);
disp(['�ڶ������������Ϊ ',num2str(Zero_Point(index(2),:))]);

% ��������ı߽�ֵ
Zero_cross_1 = Zero_Point(index(1),:);
Zero_cross_2 = Zero_Point(index(2),:);

% �ҳ��������������ȵĵ㣬������һ�߽�ֵ
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
N_2_Point_sort = sortrows(N_2_Point_sort,2);                       % ��Ҫ�Ľ�λ��һ��ĵ�

Boundary_1 = [];                                        
Boundary_2 = [];                                                   % ��ʼ���������֮����������߽�ֵ

for i=1:length(N_1_Point_sort)
    Point_i = N_1_Point_sort(i,:);
    if abs(Point_i(3))<10                                          % �����ٵ�ˮƽ����С��ĳ��Сֵʱ��ȡΪ��һ�߽�ֵ
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

disp(['ƴ�ӵ�߽�����Ϊ ',num2str(Splice_point_1)]);
disp(['ƴ�ӵ�߽�����Ϊ ',num2str(Splice_point_2)]);
disp(['ƴ�ӵ�߽�����Ϊ ',num2str(Splice_point_3)]);
disp(['ƴ�ӵ�߽�����Ϊ ',num2str(Splice_point_4)]);                  % ����ĸ��߽����꣨������������㣩

%% Bezier�� ���� Bezier ����

% ���Բ���Beizer���ߵ��㷨
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

%% PSO: ����Ⱥ�㷨

% ��ʼ����Ⱥ
[m, n]=size(x2);                                              % ��ɢ�����

N = 20;                                                       % ��ʼ��Ⱥ����
d = 2;                                                        % �ռ�ά��
ger = 500;                                                    % ����������     
limit_1 = [Splice_point_2(2), Splice_point_1(2)];            
limit_2 = [Splice_point_3(2), Splice_point_4(2)];             % ����ƴ�ӵ��λ�ò�������
limit_3 = [x_max, y_max];                                     % ����������λ�ò���Լ��
vlimit = [-1, 1];                                             % �����ٶ�����
w = 0.8;                                                      % ����Ȩ��
c1 = 0.5;                                                     % ����ѧϰ����
c2 = 0.5;                                                     % Ⱥ��ѧϰ���� 
acc = 1;                                                      % �켣����/����
x = [];                                                       % ��ʼ���Ӽ�

for i = 1:N                                                                            % ��ʼ��Ⱥ��λ��    
    x = [x; [Splice_point_1(1), limit_1(1)+(limit_1(2)-limit_1(1))*rand(1),...         % ƴ�ӵ�1
             Splice_point_3(1), limit_2(1)+(limit_2(2)-limit_2(1))*rand(1),...         % ƴ�ӵ�2
             rand(1)*x_max, 0,...                                                      % ����Ҳ��
             rand(1)*x_max, 0]];                                                       % �յ�����
end

% ���ݳ�ʼ���ӵ�λ�û�ȡ�������ٶ�ֵ
X_v = [];
for jj = 1:N
    input_x = x(jj,:);
    particle_ = particle(input_x);
    X_v = [X_v; particle_];
end
rand_point = [];                             % ƴ�ӵ�����������ĸ������
for i =1 :N
    X_splice_point_1 = X_v(i,1:4);
    X_splice_point_2 = X_v(i,5:8);
    
    Slope_1 = X_splice_point_1(4)/X_splice_point_1(3);
    Slope_2 = X_splice_point_2(4)/X_splice_point_2(3);
    
    rand_x_1_r = rand(1)*100 + X_splice_point_1(1);
    rand_y_1_r = Slope_1*(rand_x_1_r - X_splice_point_1(1)) + X_splice_point_1(2);         % ��һ��ƴ�ӵ��Ҳ�������
    rand_x_1_l = -rand(1)*100 + X_splice_point_1(1);
    rand_y_1_l = Slope_1*(rand_x_1_l - X_splice_point_1(1)) + X_splice_point_1(2);         % ��һ��ƴ�ӵ����������
    
    rand_x_2_r = rand(1)*100 + X_splice_point_2(1);
    rand_y_2_r = Slope_2*(rand_x_2_r - X_splice_point_2(1)) + X_splice_point_2(2);         % �ڶ���ƴ�ӵ��Ҳ�������
    rand_x_2_l = -rand(1)*100 + X_splice_point_2(1);
    rand_y_2_l = -Slope_2*(rand_x_2_l - X_splice_point_2(1)) + X_splice_point_2(2);        % �ڶ���ƴ�ӵ����������
    
    rand_point = [rand_point;...
                  rand_x_1_l, rand_y_1_l, rand_x_1_r, rand_y_1_r,...
                  rand_x_2_l, rand_y_2_l, rand_x_2_r, rand_y_2_r];
end
X_ALL = [X_v rand_point];

v = rand(N, d);                  % ��ʼ��Ⱥ���ٶ� 
xm = x;                          % ÿ���������ʷ���λ��
ym = zeros(1, d);                % ��Ⱥ����ʷ���λ��
fxm = zeros(N, 1);               % ÿ���������ʷ�����Ӧ��
fym = -inf;                      % ��Ⱥ��ʷ�����Ӧ��

% �����ʼ�����ӵ�λ�á��ٶ�
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
title('��ʼ����');
hold off

% ��ʼ����״̬��ʾ
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
X = [];                                % ��ý���x��y�����ݼ�
for i = 1:size_m
    X_I = X_ALL(i,:);
    X = [X; X_I(1), X_I(2), X_I(5), X_I(6), X_I(9), X_I(10), X_I(13), X_I(14)...
         X_I(17), X_I(18), X_I(19), X_I(20), X_I(21), X_I(22), X_I(23), X_I(24)];
end
X_RAW = X;
v = rand(N, 16);                       % ��ʼ��Ⱥ���ٶ�
xm = X;                                % ÿ���������ʷ���λ��
ym = zeros(1, d);                      % ��Ⱥ����ʷ���λ��
fxm = zeros(N, 1);                     % ÿ���������ʷ�����Ӧ��
fym = -inf;                            % ��Ⱥ��ʷ�����Ӧ��

%% ��Ӧ�Ⱥ���
% Ⱥ�����
% �߽�λ�ô���
limit_low = [X_RAW(1,1), limit_1(1), X_RAW(1,3), limit_2(1), -inf,  0, 0,     0, -inf,       -inf, X_RAW(1,1), -inf, -inf,       -inf, X_RAW(1,3), -inf];
limit_upp = [X_RAW(1,1), limit_1(2), X_RAW(1,3), limit_2(2), x_max, 0, x_max, 0, X_RAW(1,1), inf,  inf,        inf,  X_RAW(1,3), inf,  inf,        inf];
iter = 1;
record = zeros(ger, 1);          % ��¼��
while iter <= ger
     fx = [];
     for iii = 1:N
         fitness = fitness_self(X(iii,:));                % ���嵱ǰ��Ӧ��   
         fx = [fx fitness];                               % �ܵĸ�����Ӧ��
     end
     for i = 1:N
        if fxm(i) < fx(i)
            fxm(i) = fx(i);                               % ���¸�����ʷ�����Ӧ�� 
            xm(i,:) = X(i,:);                             % ���¸�����ʷ���λ��
        end 
     end

    if fym < max(fxm)
            [fym, nmax] = max(fxm);                        % ����Ⱥ����ʷ�����Ӧ��
            ym = xm(nmax, :);                              % ����Ⱥ����ʷ���λ��
    end

    v = v * w + c1 * rand * (xm - X) + c2 * rand * (repmat(ym, N, 1) - X);   % �ٶȸ���
    
    % �߽��ٶȴ���
    v(v > vlimit(2)) = vlimit(2);
    v(v < vlimit(1)) = vlimit(1);
    
    % λ�ø���
    X = X + v;
    
    % ��֤ƴ�ӵ������������һ������
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
    
    % λ����
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
    
    record(iter) = fym;                                       % ���ֵ��¼
%   x0 = 0 : 0.01 : 20;
%   plot(x0, f(x0), 'b-', x, f(x), 'ro');title('״̬λ�ñ仯')
%   pause(0.1)
    iter = iter+1;
end

disp(['���ֵ��',num2str(fym)]);
disp(['����ȡֵ��',num2str(ym)]);

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