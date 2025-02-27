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
error(['Unhandledflag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
global c b
sizes = simsizes;
sizes.NumContStates = 5;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 2;
sizes.NumInputs = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0 = [0*ones(5,1)];
c= [-1 -0.5 0 0.5 1;
-1 -0.5 0 0.5 1];
b=1.0;
str = [];
ts = [];
function sys=mdlDerivatives(t,x,u)
global c b
gama=1000;
yd=sin(t);
dyd=cos(t);
ddyd=-sin(t);
x1=u(2);x2=u(3);
e=yd-x1;
de=dyd-x2;
kp=30;kd=50;
K=[kp kd]';
E=[e,de]';
Fai=[0 1;-kp -kd];
A=Fai';
Q=[500 0;0 500];
P=lyap(A,Q);
xi=[x1;x2];
h=zeros(5,1);
for j=1:1:5
h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
W=[x(1) x(2) x(3) x(4) x(5)]';
B=[0;1];
S=-gama*E'*P*B*h;
for i=1:1:5
sys(i)=S(i);
end
function sys=mdlOutputs(t,x,u)
global c b
yd=sin(t);
dyd=cos(t);
ddyd=-sin(t);
x1=u(2);x2=u(3);
e=yd-x1;
de=dyd-x2;
kp=30;kd=50;
K=[kp kd]';
E=[e de]';
W=[x(1) x(2) x(3) x(4) x(5)]';
xi=[x1;x2];
h=zeros(5,1);
for j=1:1:5
h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
fxp=W'*h;
gx=133;
ut=1/gx*(-fxp+ddyd+K'*E);
sys(1)=ut;
sys(2)=fxp;