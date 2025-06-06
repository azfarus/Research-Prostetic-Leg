function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag,
case 0,
[sys,x0,str,ts]=mdlInitializeSizes;
case 1,
sys=mdlDerivatives(t,x,u);
case 3,
sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
sys = [];
otherwise
error(['Unhandledflag = ',num2str(flag)]);
end
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 2;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 4;
sizes.NumInputs = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[0.5 0];
str=[];
ts=[];
end
function sys=mdlDerivatives(t,x,u)
ut=u(1);
fx=-25*x(2)-10*x(1);
m=133;
sys(1)=x(2);
sys(2)=fx+m*ut;
end
function sys=mdlOutputs(t,x,u)
fx=-25*x(2)-10*x(1);
m=133;
sys(1)=x(1);
sys(2)=x(2);
sys(3)=fx;
sys(4)=m;
end
