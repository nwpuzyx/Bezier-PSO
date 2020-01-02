function fitness_self = fitness_self(Input_Point)
%% �������Ӹ�����Ӧ��
% �������� Beizer ���ߵĳ��ȣ�Ҫ�󳤶���̣���Ϊָ��1��ͬʱ��������� Beizer�������ߵ�ʸ���ͣ���Ϊָ��2
% �����ǽ�����x��y�����ݼ�
% ��������
load wind
x2=x(:,:,5); y2=y(:,:,5); 
u2=u(:,:,5); v2=v(:,:,5);                % ��������
[m, n] = size(x2);
w1 = 10;
w2 = 1;               % �켣���Ⱥ�����ٷ���Ĵ���ֵ����

track_fitness = 0;    % ����������Ӧ��ֵ
direc_fitness = 0;    % ����������Ӧ��ֵ
X_Beizer = [];        % Beizer �����ϵ�����
particle_v =[];       % �����ٶ����� Beizer �����ϵ�����
U = [];               % ��ʱ�洢�ٶ�ֵ

% ���� Beizer ����
t = 0:0.01:1;
xx = 0; yy = 0;
b0 = (1-t).^3;
b1 = 3 * t .* (1-t) .^2;
b2 = 3 * t .^2 .*(1-t);
b3 = t.^3;
    
x = [0, Input_Point(6), Input_Point(9)-70, Input_Point(1)-70];
y = [0, Input_Point(7), Input_Point(10)-37.5, Input_Point(2)-37.5];
xx = xx + b0*x(1) + b1*x(2) + b2*x(3) + b3*x(4);
yy = yy + b0*y(1) + b1*y(2) + b2*y(3) + b3*y(4); 
    
xx_ = 0; yy_ = 0;
x_ = [Input_Point(1)-70, Input_Point(11)-70, Input_Point(13)-70, Input_Point(3)-70];
y_ = [Input_Point(2)-37.5, Input_Point(12)-37.5, Input_Point(14)-37.5, Input_Point(4)-37.5];
xx_ = xx_ + b0*x_(1) + b1*x_(2) + b2*x_(3) + b3*x_(4);
yy_ = yy_ + b0*y_(1) + b1*y_(2) + b2*y_(3) + b3*y_(4); 

xx__ = 0; yy__ = 0;
x__ = [Input_Point(3)-70, Input_Point(15)-70, Input_Point(7), 64.3];
y__ = [Input_Point(4)-37.5, Input_Point(16)-37.5, Input_Point(8), 0];
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

% ��� Beizer �����ϵ�����
for i = 1:length(t)
    X_Beizer = [X_Beizer; xx(i), yy(i); xx_(i), yy_(i); xx__(i), yy__(i)];
end
[size_input_m, size_input_n] = size(X_Beizer);

% ��� Beizer ȥ���ϵ����꣨���ٶȣ�
for i=1:size_input_m
    Input = X_Beizer(i,:);
    Point_0 = [x2(1) y2(1) u2(1) v2(1)];
    distance_min = sqrt((Point_0(1)-Input(1))^2 + (Point_0(2)-Input(2))^2);
    index = 0;
    U = [Point_0(3), Point_0(4)];
    for ii=1:m*n
        Point_i = [x2(ii) y2(ii) u2(ii) v2(ii)];
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
[Input_m, Input_n] = size(particle_v);
for iter_i = 1:(Input_m-1)
    Particle_i = particle_v(iter_i,:);
    input_time = X_Beizer(iter_i,:);
    input_time_ = X_Beizer(iter_i+1,:);
    track_fitness = track_fitness + sqrt((input_time_(1)-input_time(1))^2 + (input_time_(2)-input_time(2))^2);
    vector = input_time_ - input_time;
    direc_fitness = direc_fitness + dot([vector(1), vector(2)], [Particle_i(3), Particle_i(4)]);
end
fitness_self = -w1 * track_fitness + w2 * direc_fitness;
end

