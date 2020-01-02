function P = bezier(x,y)
% x��y�ֱ�Ϊ����������ɵ�����
% n+1��������n��Bezier����
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