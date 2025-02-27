function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
case 0,
[sys,x0,str,ts]=mdlInitializeSizes;
case 1,
sys=mdlDerivatives(t,x,u);
case 3,
sys=mdlOutputs(t,x,u);
case {2,4,9}
sys=[];
otherwise
error(['Unhandled flag = ',num2str(flag)]);
end
end
function [sys,x0,str,ts]=mdlInitializeSizes
% global adk0 adk1 bdk1 adk2 bdk2 adk3 bdk3 adk4 bdk4 adk5 bdk5 wdk
% global ada0 ada1 bda1 ada2 bda2 ada3 bda3 ada4 bda4 ada5 bda5 wda
% %ankle coeffcient (link 2)  
%        ada0 =   -0.007782 ; %(-0.008938, -0.006626)
%        ada1 =     -0.0204  ;%(-0.02222, -0.01857)
%        bda1 =      0.0984  ;%(0.09676, 0.1)
%        ada2 =     0.01139  ;%(0.008884, 0.01389)
%        bda2 =     -0.1117  ;%(-0.1134, -0.1101)
%        ada3 =    -0.06086  ;%(-0.06254, -0.05917)
%        bda3 =     0.01065  ;%(0.008338, 0.01296)
%        ada4 =     0.03639  ;%(0.03473, 0.03805)
%        bda4 =    0.002878  ;%(0.0008274, 0.004929)
%        ada5 =    0.006763 ; %(0.004849, 0.008678)
%        bda5 =    -0.02346 ; %(-0.02511, -0.02181)
%        wda =         5.6  ;%(5.595, 5.605)
%        
%  %knee coeffcient (link 1)       
%        adk0 =      0.3912  ;
%        adk1 =     -0.1101  ;
%        bdk1 =     -0.3415  ;
%        adk2 =     -0.2344 ; 
%        bdk2 =      0.1558  ;
%        adk3 =     0.01795 ; 
%        bdk3 =     0.08599 ; 
%        adk4 =   -0.007405 ; 
%        bdk4 =   0.0006798;  
%        adk5 =    0.003116 ; 
%        bdk5 =     0.01432 ; 
%        wdk =       5.601 ;
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 6;
sizes.NumInputs = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];
end
function sys=mdlOutputs(t,x,u)
qd1=0.1*sin(t);
d_qd1=0.1*cos(t);
dd_qd1=-0.1*sin(t);
qd2=0.1*sin(t);
d_qd2=0.1*cos(t);
dd_qd2=-0.1*sin(t);
sys(1)=qd1;
sys(2)=d_qd1;
sys(3)=dd_qd1;
sys(4)=qd2;
sys(5)=d_qd2;
sys(6)=dd_qd2;
end