%��������usageSamples.m����log���ݵ������ռ�
%�����������ѹ�߶Ⱥͼ��ٶ������� �ںϸ߶���Ϣ
%

Baro_Altitude = wholeLog.SENS.BaroAlt; %ȡ����ѹ�߶�����  ���Ը߶�

Baro_Time = wholeLog.SENS.Tsec;       %��ѹ��ʱ���
Baro_Altitude(length(Baro_Altitude)) = [];  %������һ��Ԫ��ɾ����Ŀ����Ϊ��ʹ����ʱ�����ͬ
Baro_Time(length(Baro_Time)) = [];
%plot(Baro_Time,Baro_Altitude);

Acc_X = wholeLog.IMU.AccX;            %ȡ��x����ٶ�
Acc_Time = wholeLog.IMU.Tsec;         %���ٶ�ʱ���
Acc_X(length(Acc_X)) = [];            %������һ��Ԫ��ɾ����Ŀ����Ϊ��ʹ����ʱ�����ͬ
Acc_Time(length(Acc_Time)) = [];

Acc_Y = wholeLog.IMU.AccY;            %ȡ��y����ٶ�
Acc_Time = wholeLog.IMU.Tsec;         %���ٶ�ʱ���
Acc_Y(length(Acc_Y)) = [];            %������һ��Ԫ��ɾ����Ŀ����Ϊ��ʹ����ʱ�����ͬ
Acc_Time(length(Acc_Time)) = [];

Acc_Z = wholeLog.IMU.AccZ;            %ȡ��z����ٶ�
Acc_Time = wholeLog.IMU.Tsec;         %���ٶ�ʱ���
Acc_Z(length(Acc_Z)) = [];            %������һ��Ԫ��ɾ����Ŀ����Ϊ��ʹ����ʱ�����ͬ
Acc_Time(length(Acc_Time)) = [];

Roll = wholeLog.ATT.Roll;             %��̬
Pitch = wholeLog.ATT.Pitch;
Yaw = wholeLog.ATT.Yaw;
ATT_Time = wholeLog.ATT.Tsec;         %��̬ʱ���

Time_temp = Baro_Time(1);
Baro_Time = Baro_Time - Time_temp;    %��ѹ��ʱ����㿪ʼ

Time_temp = Acc_Time(1);
Acc_Time = Acc_Time - Time_temp;     %���ٶ�ʱ����㿪ʼ

Time_temp = ATT_Time(1);
ATT_Time = ATT_Time - Time_temp;     %��̬ʱ����㿪ʼ  

acc_bias = [0.0 0.0 0.0];           %���ٶ�ƫ��  ���� 
corr_baro = 0.0;
acc = [0 0 0];                      %���� 
z_est = [0 0];                      %z��λ�ú��ٶ� 
accel_bias_corr = [0 0 0];          %���ٶ�ƫ��У��ֵ
w_acc_bias = 0.05;                  % 0 - 0.1
w_z_baro = 0.5;                     % 0 - 10

P = Pitch(1);
R = Roll(1);
Y = Yaw(1);

Rot = [cos(P)*cos(Y) sin(P)*sin(R)*cos(Y)-sin(Y)*cos(R) sin(P)*cos(R)*cos(Y)+sin(Y)*sin(R);   %���ᵽ����
       cos(P)*sin(Y) sin(Y)*sin(P)*sin(R)+cos(Y)*cos(R) sin(Y)*sin(P)*cos(R)-cos(Y)*sin(R);
       -sin(P)                     sin(R)*cos(P)                    cos(R)*cos(P)];

Baro_Sum = 0;
% plot(Baro_Time,Baro_Altitude)
for(counter = 1:1:200)                                 % ȡ���ٸ�ƽ��ֵ  �����׼�߶�
    Baro_Sum = Baro_Sum + Baro_Altitude(counter);
end
Baro_Offset = Baro_Sum / counter;

 for(i = 1:1:length(Acc_Time))

    %�ж���ѹ���Ƿ����
   for(j = 1:1:length(Baro_Time))
     if( Baro_Time(j) == Acc_Time(i))
          Baro_Altitude_Update = 1;
          Baro_Altitude_Now = Baro_Altitude(j);  %ȡ����ǰʱ�̵���ѹ�߶ȣ����ڼ���
          break;
      end
     if(Baro_Time(j) > Acc_Time(i))
          Baro_Altitude_Update = 0;
          break;
     end
   end
   %��ѹ�����жϽ���
   %��ʼ��̬�����ж�
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
   %��̬�����жϽ���
   
   if(ATT_Update)   %���ᵽ����
    Rot = [cos(P)*cos(Y) sin(P)*sin(R)*cos(Y)-sin(Y)*cos(R) sin(P)*cos(R)*cos(Y)+sin(Y)*sin(R);   %���ᵽ����
           cos(P)*sin(Y) sin(Y)*sin(P)*sin(R)+cos(Y)*cos(R) sin(Y)*sin(P)*cos(R)-cos(Y)*sin(R);   %������ת����
           -sin(P)                     sin(R)*cos(P)                    cos(R)*cos(P)         ];
   end
   
   %�㷨��ʼ
   Acc_X(i) = Acc_X(i) - acc_bias(1);    %������ٶ� ��ȥ ���ٶ�ƫ��
   Acc_Y(i) = Acc_Y(i) - acc_bias(2);
   Acc_Z(i) = Acc_Z(i) - acc_bias(3);
   
   acc = Rot * [Acc_X(i);Acc_Y(i);Acc_Z(i)];   %������ٶ�ת�Ƶ� ����
   acc(3) = acc(3) + 9.80665;           %���ٶ�ԭʼ����ֵΪ ����+9.8�� ʣ�ಿ��Ϊ�����˶������ļ��ٶ�  �� -9,8
   
   if(Baro_Altitude_Update)
       corr_baro = Baro_Offset - Baro_Altitude_Now - z_est(1);
   end
   %�����ٶ�ƫ��������� �ɵ��� ת�Ƶ�����
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
  
  accel_bias_corr(3) = accel_bias_corr(3) - corr_baro * w_z_baro * w_z_baro;  %��corr_baro������ٶ�ƫ�ƽ�����
  
   %���ˣ�ʹ��dt z_est acc(3) ��ʼ Ԥ��
   
   z_est(1) = z_est(1) + z_est(2) * dt + acc(3) * dt * dt / 2.0;
   z_est(2) = z_est(2) + acc(3) * dt;
   
   %��ʼ����  ʹ��corr_baro  dt  z_est 0 w_z_baro
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

