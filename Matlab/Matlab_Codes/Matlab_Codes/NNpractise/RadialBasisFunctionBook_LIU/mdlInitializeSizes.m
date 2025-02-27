function [sys,x0,str,ts]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=0;
sizes.NumDiscStates=0;
sizes.NumOutputs=7;
sizes.NumInputs=1;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=0;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[];
end