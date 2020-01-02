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

x_ = [2 3 4 5];
y_ = [-1 -1 1 1];
xx_ = xx_0 + b0*x_(1) + b1*x_(2) + b2*x_(3) + b3*x_(4);
yy_ = yy_0 + b0*y_(1) + b1*y_(2) + b2*y_(3) + b3*y_(4); 

x__ = [5 9 4 8];
y__ = [1 1 0 0];
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