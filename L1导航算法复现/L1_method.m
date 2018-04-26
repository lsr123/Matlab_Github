%L1导航算法
%思路
% 由上到下的步骤
% 1. 当前点的速度V
% 2.求V与X轴的夹角
% 3.求当前点的加速度a_cmd
% 4.将加速度z_cmd分解到X轴的Y轴
% 5.计算下一点的速度V_x 和 V_y
% 6.由当前位置计算下一点的位置x_plane 和 y_plane
clear
%步长
dt = 0.001;

%定义跟踪路径  半径R=250的  圆心坐标（0,0）
x_target =  -250:0.1:250;
y_target = sqrt(250^2 - x_target.^2);


%导航半径 250 参数
R = 250;
L1 =  150;
c = sqrt(1 - (L1/(2*R))^2);
%定义飞行器初始位置速度
V = 25; %初始速度
V_x = [];
V_y = [];
V_x(1) = 0;
V_y(1) = 25;
x_plane = [];
y_plane = [];
x_plane(1) = -250;
y_plane(1) = 0;

alpha = [];
d = [];
d_dot = [];
d_last = 0;  %上一次的航迹偏差

a_cmd = [];
a_x = [];
a_y = [];

for i=1:50000
  %alpha(i) = acos((V_x(i)/sqrt(V_x(i)*V_x(i) + V_y(i)*V_y(i))));  %计算当前位置 速度 与X轴的夹角
  alpha(i) = asin((V_y(i)/sqrt(V_x(i)*V_x(i) + V_y(i)*V_y(i))));  %计算当前位置 速度 与X轴的夹角  步骤2
  %acos(V_x(i)/sqrt((V_x(i)*V_x(i) + V_y(i)*V_y(i))));
  
  d(i) = sqrt(x_plane(i)^2 + y_plane(i)^2) - R;%得到航迹偏差d 
  d_dot(i) = (d(i) - d_last)/dt; %航迹偏差导数
  a_cmd(i) = 2*V^2*c^2*d(i)/L1^2 + 2*V*c/L1*d_dot(i) + V^2/R;  %步骤3
  
  %判断一下加速度的方向
  %a_x(i) = a_cmd(i) * sin(alpha(i));
  
  a_x(i) = a_cmd(i) * sin(alpha(i)); %a_x 与V_x同向     步骤4
  a_y(i) = -a_cmd(i) * cos(alpha(i)); %a_y与V_y 反向 加负号
  
  V_x(i+1) = V_x(i) + a_x(i)*dt;      %计算下一个点的速度  步骤5
  V_y(i+1) = V_y(i) + a_y(i)*dt;
  
  x_plane(i+1) = x_plane(i) + (V_x(i) + V_x(i+1))*dt/2; %步骤6
  y_plane(i+1) = y_plane(i) + (V_y(i) + V_y(i+1))*dt/2;
  
  if x_plane(i)>250  %适当地跳出循环
      break;
  end
end

plot(x_plane,y_plane)
hold on
plot(x_target,y_target);
