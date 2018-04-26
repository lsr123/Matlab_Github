%L1�����㷨
%˼·
% ���ϵ��µĲ���
% 1. ��ǰ����ٶ�V
% 2.��V��X��ļн�
% 3.��ǰ��ļ��ٶ�a_cmd
% 4.�����ٶ�z_cmd�ֽ⵽X���Y��
% 5.������һ����ٶ�V_x �� V_y
% 6.�ɵ�ǰλ�ü�����һ���λ��x_plane �� y_plane
clear
%����
dt = 0.001;

%�������·��  �뾶R=250��  Բ�����꣨0,0��
x_target =  -250:0.1:250;
y_target = sqrt(250^2 - x_target.^2);


%�����뾶 250 ����
R = 250;
L1 =  150;
c = sqrt(1 - (L1/(2*R))^2);
%�����������ʼλ���ٶ�
V = 25; %��ʼ�ٶ�
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
d_last = 0;  %��һ�εĺ���ƫ��

a_cmd = [];
a_x = [];
a_y = [];

for i=1:50000
  %alpha(i) = acos((V_x(i)/sqrt(V_x(i)*V_x(i) + V_y(i)*V_y(i))));  %���㵱ǰλ�� �ٶ� ��X��ļн�
  alpha(i) = asin((V_y(i)/sqrt(V_x(i)*V_x(i) + V_y(i)*V_y(i))));  %���㵱ǰλ�� �ٶ� ��X��ļн�  ����2
  %acos(V_x(i)/sqrt((V_x(i)*V_x(i) + V_y(i)*V_y(i))));
  
  d(i) = sqrt(x_plane(i)^2 + y_plane(i)^2) - R;%�õ�����ƫ��d 
  d_dot(i) = (d(i) - d_last)/dt; %����ƫ���
  a_cmd(i) = 2*V^2*c^2*d(i)/L1^2 + 2*V*c/L1*d_dot(i) + V^2/R;  %����3
  
  %�ж�һ�¼��ٶȵķ���
  %a_x(i) = a_cmd(i) * sin(alpha(i));
  
  a_x(i) = a_cmd(i) * sin(alpha(i)); %a_x ��V_xͬ��     ����4
  a_y(i) = -a_cmd(i) * cos(alpha(i)); %a_y��V_y ���� �Ӹ���
  
  V_x(i+1) = V_x(i) + a_x(i)*dt;      %������һ������ٶ�  ����5
  V_y(i+1) = V_y(i) + a_y(i)*dt;
  
  x_plane(i+1) = x_plane(i) + (V_x(i) + V_x(i+1))*dt/2; %����6
  y_plane(i+1) = y_plane(i) + (V_y(i) + V_y(i+1))*dt/2;
  
  if x_plane(i)>250  %�ʵ�������ѭ��
      break;
  end
end

plot(x_plane,y_plane)
hold on
plot(x_target,y_target);
