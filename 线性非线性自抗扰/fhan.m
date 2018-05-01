function u = fhan(x1,x2,r,h)
%FHAN 最速控制综合函数
%   此处显示详细说明
d = r*h;
d0 = h*d;
y = x1+h*x2;
a0 = sqrt(d*d+8*r*abs(y));

if abs(y)>d0
    a = x2+(a0-d)/2*sign(y);
end
if abs(y)<=d0
    a = x2+y/h; 
end

if abs(a)>d
    u = -r*sign(a);
end
if abs(a)<=d
    u = -r*a/d;
end

end

