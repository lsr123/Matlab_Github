startData = 10; %9000;  %��ȡ�������ݵ���ʼλ��
stopData = 1699;%16999;  %��ȡ�������ݵ�ֹͣλ��  ���������Ҫ������
t = 0:0.01:169.89;
%S = 0.7*sin(2*pi*20*t) + sin(2*pi*30*t);   %���ɵ��������ݣ����ڼ���������ȷ��

pitchAll = attitudeData.ATT.Pitch;
pitch = pitchAll(startData:stopData);

%pitch = pitch +S;

L = stopData - startData + 1;  %���ݳ���
Fs = 100;  % ������
Y = fft(pitch);     %��pitch ���ٸ���Ҷչ��


P2 = abs(Y/L);         % Compute the two-sided spectrum P2. 
P1 = P2(1:L/2+1);      % Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;   %
plot(f,P1)            %����û�����˲� ��ԭʼ���ݵ�Ƶ��ͼ
hold on
                                    %_filter��β��Ϊ �˲�֮��Ķ��� 
pitch_filter = filter(Hd,pitch); % Hd Ϊ�� �˲�����Ʒ��������� ����������ռ���˲�����
YY = fft(pitch_filter);          %�˲�֮������ݿ��ٸ���Ҷչ��

P2_filter = abs(YY/L);         % Compute the two-sided spectrum P2. 
P1_filter = P2_filter(1:L/2+1);      % Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L
P1_filter(2:end-1) = 2*P1_filter(2:end-1);

tt = 0:0.01:245.17;
plot(f,P1_filter)

% for i = 1:24518               %��Ϊ ������ʱ��������ǹ̶��ģ��������˲�ֵ�ķ�����ʵ�� ��ͬʱ�� ���
%     for j = 1:24517
%         if tt(i)>Time(j)&tt(i)<Time(j+1)
%             pitch_inter(i) = (pitchAll(j+1)-pitchAll(j))/(Time(j+1)-Time(j))*(tt(i) - Time(j)) + pitchAll(j);
%             break;
%         end
%         
%     end
% end

