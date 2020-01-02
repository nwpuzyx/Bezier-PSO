function particle_v = particle(input)
% 利用格栅数据获取，任意粒子的速度
% 输入点的坐标（仅含x和y），输出含有速度量的坐标值
load wind                     
x2=x(:,:,5); y2=y(:,:,5); 
u2=u(:,:,5); v2=v(:,:,5);                % 导入数据
[m, n] = size(x2);
[input_m, input_n] = size(input);
Output = [];
particle_v = [];
U = [];
for ii = 1:(input_n/2)
    Input = [input(:,2*ii-1), input(:,2*ii)];
    index = 0;
    distance_min = sqrt(Input(1)^2 + Input(2)^2);
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
    particle_v = [particle_v, Output];
end

