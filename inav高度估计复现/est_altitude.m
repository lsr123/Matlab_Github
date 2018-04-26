%首先运行usageSamples.m解析log数据到工作空间
%这个函数用气压高度和加速度数据来 融合高度信息
%

Baro_Altitude = wholeLog.SENS.BaroAlt; %取得气压高度数据  绝对高度

Baro_Time = wholeLog.SENS.Tsec;       %气压计时间戳
Baro_Altitude(length(Baro_Altitude)) = [];  %将最有一个元素删除，目的是为了使最后的时间点相同
Baro_Time(length(Baro_Time)) = [];
%plot(Baro_Time,Baro_Altitude);

Acc_X = wholeLog.IMU.AccX;            %取得x轴加速度
Acc_Time = wholeLog.IMU.Tsec;         %加速度时间戳
Acc_X(length(Acc_X)) = [];            %将最有一个元素删除，目的是为了使最后的时间点相同
Acc_Time(length(Acc_Time)) = [];

Acc_Y = wholeLog.IMU.AccY;            %取得y轴加速度
Acc_Time = wholeLog.IMU.Tsec;         %加速度时间戳
Acc_Y(length(Acc_Y)) = [];            %将最有一个元素删除，目的是为了使最后的时间点相同
Acc_Time(length(Acc_Time)) = [];

Acc_Z = wholeLog.IMU.AccZ;            %取得z轴加速度
Acc_Time = wholeLog.IMU.Tsec;         %加速度时间戳
Acc_Z(length(Acc_Z)) = [];            %将最有一个元素删除，目的是为了使最后的时间点相同
Acc_Time(length(Acc_Time)) = [];

Roll = wholeLog.ATT.Roll;             %姿态
Pitch = wholeLog.ATT.Pitch;
Yaw = wholeLog.ATT.Yaw;
ATT_Time = wholeLog.ATT.Tsec;         %姿态时间戳

Time_temp = Baro_Time(1);
Baro_Time = Baro_Time - Time_temp;    %气压计时间从零开始

Time_temp = Acc_Time(1);
Acc_Time = Acc_Time - Time_temp;     %加速度时间从零开始

Time_temp = ATT_Time(1);
ATT_Time = ATT_Time - Time_temp;     %姿态时间从零开始  

acc_bias = [0.0 0.0 0.0];           %加速度偏移  体轴 
corr_baro = 0.0;
acc = [0 0 0];                      %地轴 
z_est = [0 0];                      %z轴位置和速度 
accel_bias_corr = [0 0 0];          %加速度偏移校正值
w_acc_bias = 0.05;                  % 0 - 0.1
w_z_baro = 0.5;                     % 0 - 10

P = Pitch(1);
R = Roll(1);
Y = Yaw(1);

Rot = [cos(P)*cos(Y) sin(P)*sin(R)*cos(Y)-sin(Y)*cos(R) sin(P)*cos(R)*cos(Y)+sin(Y)*sin(R);   %体轴到地轴
       cos(P)*sin(Y) sin(Y)*sin(P)*sin(R)+cos(Y)*cos(R) sin(Y)*sin(P)*cos(R)-cos(Y)*sin(R);
       -sin(P)                     sin(R)*cos(P)                    cos(R)*cos(P)];

Baro_Sum = 0;
% plot(Baro_Time,Baro_Altitude)
for(counter = 1:1:200)                                 % 取两百个平均值  计算基准高度
    Baro_Sum = Baro_Sum + Baro_Altitude(counter);
end
Baro_Offset = Baro_Sum / counter;

 for(i = 1:1:length(Acc_Time))

    %判断气压计是否更新
   for(j = 1:1:length(Baro_Time))
     if( Baro_Time(j) == Acc_Time(i))
          Baro_Altitude_Update = 1;
          Baro_Altitude_Now = Baro_Altitude(j);  %取出当前时刻的气压高度，用于计算
          break;
      end
     if(Baro_Time(j) > Acc_Time(i))
          Baro_Altitude_Update = 0;
          break;
     end
   end
   %气压更新判断结束
   %开始姿态更新判断
   for(k = 1:1:length(ATT_Time))
       if(ATT_Time(k) == Acc_Time(i))
           ATT_Update = 1;
           R = Roll(k);
           P = Pitch(k);
           Y = Yaw(k);
          
           break;
       end
       if(ATT_Time(k) > Acc_Time(i))
           ATT_Update = 0;
           break;
       end
   end   
   %姿态更新判断结束
   
   if(ATT_Update)   %体轴到地轴
    Rot = [cos(P)*cos(Y) sin(P)*sin(R)*cos(Y)-sin(Y)*cos(R) sin(P)*cos(R)*cos(Y)+sin(Y)*sin(R);   %体轴到地轴
           cos(P)*sin(Y) sin(Y)*sin(P)*sin(R)+cos(Y)*cos(R) sin(Y)*sin(P)*cos(R)-cos(Y)*sin(R);   %更新旋转矩阵
           -sin(P)                     sin(R)*cos(P)                    cos(R)*cos(P)         ];
   end
   
   %算法开始
   Acc_X(i) = Acc_X(i) - acc_bias(1);    %体轴加速度 减去 加速度偏移
   Acc_Y(i) = Acc_Y(i) - acc_bias(2);
   Acc_Z(i) = Acc_Z(i) - acc_bias(3);
   
   acc = Rot * [Acc_X(i);Acc_Y(i);Acc_Z(i)];   %体轴加速度转移到 地轴
   acc(3) = acc(3) + 9.80665;           %加速度原始测量值为 负，+9.8后 剩余部分为机身运动产生的加速度  减 -9,8
   
   if(Baro_Altitude_Update)
       corr_baro = Baro_Offset - Baro_Altitude_Now - z_est(1);
   end
   %将加速度偏移误差向量 由地轴 转移到体轴
  c = [0.0 0.0 0.0];
  c = Rot'* accel_bias_corr';
  if(i > 1)
  dt = Acc_Time(i) - Acc_Time(i-1);
  else
      dt = 0;
  end
  acc_bias(1) = acc_bias(1) + c(1) * w_acc_bias * dt;
  acc_bias(2) = acc_bias(2) + c(2) * w_acc_bias * dt;
  acc_bias(3) = acc_bias(3) + c(3) * w_acc_bias * dt;
  
  accel_bias_corr(1) = 0.0;
  accel_bias_corr(2) = 0.0;
  accel_bias_corr(3) = 0.0;
  
  accel_bias_corr(3) = accel_bias_corr(3) - corr_baro * w_z_baro * w_z_baro;  %由corr_baro计算加速度偏移矫正量
  
   %至此，使用dt z_est acc(3) 开始 预测
   
   z_est(1) = z_est(1) + z_est(2) * dt + acc(3) * dt * dt / 2.0;
   z_est(2) = z_est(2) + acc(3) * dt;
   
   %开始矫正  使用corr_baro  dt  z_est 0 w_z_baro
   ewdt = corr_baro * w_z_baro * dt;
   z_est(1) = z_est(1) + ewdt;
   z_est(2) = z_est(2) + w_z_baro * ewdt;
   
  height(i) = z_est(1);
 end
  
Lpos_Z = wholeLog.LPOS.Z;
Lpos_Z_Time = wholeLog.LPOS.Tsec;
Lpos_Z_Time_Temp = Lpos_Z_Time(1);
Lpos_Z_Time = Lpos_Z_Time - Lpos_Z_Time_Temp;
 plot(Acc_Time, height ,Lpos_Z_Time,Lpos_Z ,Baro_Time,-(Baro_Altitude -365.6394));

