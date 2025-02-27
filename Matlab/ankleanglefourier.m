% close all
% clear all
% clc
%
% % Read the Th1, Th2, calculate the ThA, then approximate using Fourier
%
% % CHANGE BETWEEN GAIT TYPE: SLOW, NORMAL, AND FAST
% gait = 'Normal'; % or 'Normal', or 'Fast' or 'Slow'
%
% data = xlsread('Gait Data - Patterns.xlsx',[gait,'(Winter,1991)']);
% data = data(4:54,:);
% t = [data(:,4)
%     data(:,4)+ max(data(:,4)) + data(2,4)
%     data(:,4) + 2*max(data(:,4)) + 2*data(2,4)];
% ThA = [data(:,9);data(:,9);data(:,9)]*pi/180;
%
% figure('Name','Ankle angle in radian')
% plot(t,ThA)
%
% disp('FIT THE ANKLE ANGLE USING FOURIER 5 SERIES, RUN cftool IN MATLAB')
% cftool(t,ThA)

% %
% clear all; close all; clc;
% gait = 'Normal'; % or 'Normal', or 'Fast'
%
% data = xlsread('Gait Data - Patterns.xlsx',[gait,'(Winter,1991)']);
% data = data(4:54,:);
% t = [data(:,4)
%     data(:,4)+ max(data(:,4)) + data(2,4)
%     data(:,4) + 2*max(data(:,4)) + 2*data(2,4)];
% Thk = [data(:,8);data(:,8);data(:,8)]*pi/180;
%
% figure('Name','Ankle angle in radian')
% plot(t,Thk)
%
% disp('FIT THE ANKLE ANGLE USING FOURIER 5 SERIES, RUN cftool IN MATLAB')
% cftool(t,Thk)

%%

% %%
% clear all; close all; clc;
% gait = 'Normal'; % or 'Normal', or 'Fast'
%
% data = xlsread('Gait Data - Patterns.xlsx',[gait,'(Winter,1991)']);
% data = data(4:54,:);
% t = [data(:,4)
%     data(:,4)+ max(data(:,4)) + data(2,4)
%     data(:,4) + 2*max(data(:,4)) + 2*data(2,4)];
% Thk = [data(:,7);data(:,7);data(:,7)]*pi/180;
%
% figure('Name','Hip angle in radian')
% plot(t,Thk)
%
% disp('FIT THE Hip ANGLE USING FOURIER 5 SERIES, RUN cftool IN MATLAB')
% cftool(t,Thk)
%%

%%
clear all; close all; clc;
gait = 'Joint Rotations'; % or 'Normal', or 'Fast'

data = xlsread('anglehipgait.xls', gait);
Thhip = data(52:102, 9) * pi / 180;;
% t = [data(:,4)
%     data(:,4)+ max(data(:,4)) + data(2,4)
%     data(:,4) + 2*max(data(:,4)) + 2*data(2,4)];
t = data(52:102, 1);
% Thk = [data(:,7);data(:,7);data(:,7)]*pi/180;
%% Thk = [data(:,7);data(:,7);data(:,7)]*pi/180;
% figure('Name','Hip angle in radian')
% plot(t,Thk)
%
disp('FIT THE Hip ANGLE USING FOURIER 5 SERIES, RUN cftool IN MATLAB')
cftool(t, Thhip)
