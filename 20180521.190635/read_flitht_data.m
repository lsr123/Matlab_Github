clear
fid = fopen('20180513成功飞行数据.txt','r');
bb = textscan(fid,'%s');
fclose(fid);

data_num = 45;  %共有45个条目

temp = size(bb{1,1});
count_num = temp(1)/45;
hwait = waitbar(0,'running');
for i = 1:count_num-1        %表头不要
    
    count(i) = bb{1,1}(i*45+1,1);
    count_out = cellfun(@str2num,count);
    
    nowSysTimeinMS(i) = bb{1,1}(i*45+2,1);
    nowSysTimeinMS_out =  cellfun(@str2num,nowSysTimeinMS);
    
    flightMode(i) = bb{1,1}(i*45+10,1);
    flightMode_out = cellfun(@str2num,flightMode);
    
    bankcmd(i) = bb{1,1}(i*45+25,1);
    bankcmd_out = cellfun(@str2num,bankcmd);
    
    bankAngle(i) = bb{1,1}(i*45+26,1);
    bankAngle_out = cellfun(@str2num,bankAngle);
    
    pitchcmd(i) = bb{1,1}(i*45+27,1);
    pitchcmd_out = cellfun(@str2num,pitchcmd);
    
    pitchAngle(i) = bb{1,1}(i*45+28,1);
    pitchAngle_out = cellfun(@str2num,pitchAngle);
    
    headingcmd(i) = bb{1,1}(i*45+29,1);
    headingcmd_out = cellfun(@str2num,headingcmd);
    
    headingAngle(i) = bb{1,1}(i*45+30,1);
    headingAngle_out = cellfun(@str2num,headingAngle);
    
    altitudecmd(i) = bb{1,1}(i*45+31,1);
    altitudecmd_out = cellfun(@str2num,altitudecmd);
    
    altitude(i) = bb{1,1}(i*45+32,1);
    altitude_out = cellfun(@str2num,altitude);
    
    hdotcmd(i) = bb{1,1}(i*45+33,1);
    hdotcmd_out = cellfun(@str2num,hdotcmd);
    
    hdot(i) = bb{1,1}(i*45+34,1);
    hdot_out = cellfun(@str2num,hdot);
    
    tasCmd(i) = bb{1,1}(i*45+35,1);
    tasCmd_out = cellfun(@str2num,tasCmd);
    
    tas(i) = bb{1,1}(i*45+36,1);
    tas_out = cellfun(@str2num,tas);
    
    tasdotCmd(i) = bb{1,1}(i*45+37,1);
    tasdotCmd_out = cellfun(@str2num,tasdotCmd);
    
    tasdot(i) = bb{1,1}(i*45+38,1);
    tasdot_out = cellfun(@str2num,tasdot);
    
    aileronLeftCmd(i) = bb{1,1}(i*45+39,1);
    aileronLeftCmd_out = cellfun(@str2num,aileronLeftCmd);
    
    elevatorCmd = bb{1,1}(i*45+40,1);
    elevatorCmd_out = cellfun(@str2num,elevatorCmd);
    
    throttleCmd = bb{1,1}(i*45+41,1);
    throttleCmd_out = cellfun(@str2num,throttleCmd);
    
    rudderCmd = bb{1,1}(i*45+42,1);
    rudderCmd_out = cellfun(@str2num,rudderCmd);
    
    str = ['运行中',num2str(floor(i/count_num*100)),'%'];
   waitbar(i/count_num,hwait,str);
end
close(hwait);