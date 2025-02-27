 
function [sys,x0,str,ts]=spacemodel(t,x,u,flag) 
switch flag,
case 0,
[sys,x0,str,ts]=mdlInitializeSizes;
case 3,
sys=mdlOutputs(t,x,u);
case {2,4,9}
sys=[];
otherwise
error(['Unhandled flag =', num2str(flag)]) 
end
end