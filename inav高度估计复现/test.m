startData = 10; %9000;  %截取飞行数据的起始位置
stopData = 1699;%16999;  %截取飞行数据的停止位置  两数据相减要是奇数
t = 0:0.01:169.89;
%S = 0.7*sin(2*pi*20*t) + sin(2*pi*30*t);   %生成的虚拟数据，用于检验程序的正确性

pitchAll = attitudeData.ATT.Pitch;
pitch = pitchAll(startData:stopData);

%pitch = pitch +S;

L = stopData - startData + 1;  %数据长度
Fs = 100;  % 采样率
Y = fft(pitch);     %对pitch 快速傅里叶展开


P2 = abs(Y/L);         % Compute the two-sided spectrum P2. 
P1 = P2(1:L/2+1);      % Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;   %
plot(f,P1)            %画出没经过滤波 的原始数据的频谱图
hold on
                                    %_filter结尾的为 滤波之后的东西 
pitch_filter = filter(Hd,pitch); % Hd 为由 滤波器设计分析工具箱 输出到工作空间的滤波函数
YY = fft(pitch_filter);          %滤波之后的数据快速傅里叶展开

P2_filter = abs(YY/L);         % Compute the two-sided spectrum P2. 
P1_filter = P2_filter(1:L/2+1);      % Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L
P1_filter(2:end-1) = 2*P1_filter(2:end-1);

tt = 0:0.01:245.17;
plot(f,P1_filter)

% for i = 1:24518               %因为 采样的时间点间隔不是固定的，所以用了插值的方法来实现 相同时间 间隔
%     for j = 1:24517
%         if tt(i)>Time(j)&tt(i)<Time(j+1)
%             pitch_inter(i) = (pitchAll(j+1)-pitchAll(j))/(Time(j+1)-Time(j))*(tt(i) - Time(j)) + pitchAll(j);
%             break;
%         end
%         
%     end
% end

