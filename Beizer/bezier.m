function P = bezier(x,y)
% x和y分别为横纵坐标组成的向量
% n+1个点生成n次Bezier曲线
n = length(x);
t = linspace(0,1);
Px = 0;
Py = 0;
for k = 0:n-1
    B_i_n = nchoosek(n-1,k).*t.^k.*(1-t).^(n-1-k);
    Px = Px + x(k+1)*B_i_n;
    Py = Py + y(k+1)*B_i_n;
end
plot(Px,Py);