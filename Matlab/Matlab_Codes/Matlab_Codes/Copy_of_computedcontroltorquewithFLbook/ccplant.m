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
error(['Unhandled flag = ',num2str(flag)]);
end
end
function [sys,x0,str,ts]=mdlInitializeSizes
global Kv Kp
global  g m1 m2 a1 a2
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
Kv=20*eye(2);%%Kv 
Kp = 100*eye(2) ; %Kp
sizes = simsizes;
sizes.NumContStates = 4;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 4;%
sizes.NumInputs =2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys=simsizes(sizes);
x0=[0 0 0 0];
str=[];
ts=[0 0];
% p=[2.9 0.76 0.87 3.04 0.87];
g=9.8;
m1 = 2.6366;%5 ; 
m2 = 0.82,%2.6366+0.82 ;
a1 = 0.19,%0.38 ;
a2 = 0.0608,%0.0608+0.38 ;
g=9.8;
end
function sys=mdlDerivatives(t,x,u)
global g m1 m2 a1 a2 

% global g m1 m2 a1 a2 

D = [(m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(3)), m2*a2^2 + m2*a1*a2*cos(x(3)); m2*a2^2 + m2*a1*a2*cos(x(3)), m2*a2^2];
C = [ -m2*a1*a2*(2*x(2)*x(4)+(x(4))^2)*sin(x(3));m2*a1*a2*(x(2))^2*sin(x(3))] ; 
G = [(m1+m2)*g*a1*cos(x(1))+m2*g*a2*cos(x(1)+x(3)); m2*g*a2*cos(x(1)+x(3))]; 
dq=[x(2);x(4)];
% F=0*0.02*sign(dq);%
% told=0*[0.1*sin(t);0.1*sin(t)];%
tol=u(1:2);
S=inv(D)*(tol-C-G);%
sys(1)=x(2);
sys(2)=S(1);
sys(3)=x(4);
sys(4)=S(2);
end
function sys=mdlOutputs(t,x,u)
% global Kv Kp 
% global adk0 adk1 bdk1 adk2 bdk2 adk3 bdk3 adk4 bdk4 adk5 bdk5 wdk
% global ada0 ada1 bda1 ada2 bda2 ada3 bda3 ada4 bda4 ada5 bda5 wda
global g m1 m2 a1 a2
D = [(m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(3)), m2*a2^2 + m2*a1*a2*cos(x(3)); m2*a2^2 + m2*a1*a2*cos(x(3)), m2*a2^2];
C = [ -m2*a1*a2*(2*x(2)*x(4)+(x(4))^2)*sin(x(3));m2*a1*a2*(x(2))^2*sin(x(3))] ; 
G = [(m1+m2)*g*a1*cos(x(1))+m2*g*a2*cos(x(1)+x(3)); m2*g*a2*cos(x(1)+x(3))]; 
% dq=[x(2);x(4)];
% F=0.2*sign(dq);
% told=[0.1*sin(t);0.1*sin(t)];
% N = C + G %+F +told ; 
% dq=[x(2);x(4)];
% F=0.2*sign(dq);
% told=[0.1*sin(t);0.1*sin(t)];
% N = C + G +F +told ; 
% dq=[x(2);x(4)];
% F=0.2*sign(dq);
% told=[0.1*sin(t);0.1*sin(t)];
%------knee ----link1------------
% qd1= adk0 + adk1*cos(wdk*t) + bdk1*sin(wdk*t) +  adk2*cos(2*wdk*t) +...
%     bdk2*sin(2*wdk*t) + adk3*cos(3*wdk*t) + bdk3*sin(3*wdk*t)+adk4*cos(4*wdk*t) +...
%     bdk4*sin(4*wdk*t) +adk5*cos(5*wdk*t) + bdk5*sin(5*wdk*t) ;
% 
% d_qd1= -adk1*wdk*sin(wdk*t) + bdk1*wdk*cos(wdk*t) -  adk2*2*wdk*sin(2*wdk*t) +...
%     bdk2*2*wdk*cos(2*wdk*t) - adk3*3*wdk*sin(3*wdk*t) + bdk3*3*wdk*cos(3*wdk*t)- adk4*4*wdk*sin(4*wdk*t)+...
%     4*wdk*bdk4*cos(4*wdk*t) -adk5*5*wdk*sin(5*wdk*t) + bdk5*5*wdk*cos(5*wdk*t) ;
% 
% 
% dd_qd1= - adk1*wdk^2*cos(wdk*t) - bdk1*wdk^2*sin(wdk*t) -  adk2*wdk^2*4*cos(2*wdk*t) -...
%     bdk2*wdk^2*4*sin(2*wdk*t) - adk3*wdk^2*9*cos(3*wdk*t) - bdk3*wdk^2*9*sin(3*wdk*t)-adk4*wdk^2*16*cos(4*wdk*t) -...
%     bdk4*wdk^2*16*sin(4*wdk*t) -adk5*wdk^2*25*cos(5*wdk*t) - bdk5*wdk^2*25*sin(5*wdk*t) ;
% 
% 
% %ankle--link2 
% qd2= ada0 + ada1*cos(wda*t) + bda1*sin(wda*t) +  ada2*cos(2*wda*t) +...
%     bda2*sin(2*wda*t) + ada3*cos(3*wda*t) + bda3*sin(3*wda*t)+ada4*cos(4*wda*t) +...
%     bda4*sin(4*wda*t) +ada5*cos(5*wda*t) + bda5*sin(5*wda*t) ;
% 
% d_qd2=-ada1*wda*sin(wda*t) + bda1*wda*cos(wda*t) -  ada2*2*wda*sin(2*wda*t) +...
%     bda2*2*wda*cos(2*wda*t) - ada3*3*wda*sin(3*wda*t) - bda3*3*wda*cos(3*wda*t)- ada4*4*wda*sin(4*wda*t)+...
%     4*wda*bda4*cos(4*wda*t) -ada5*5*wda*sin(5*wda*t) + bda5*5*wda*cos(5*wda*t) ;
% 
% dd_qd2= -ada1*wda^2*cos(wda*t) - bda1*wda^2*sin(wda*t) -  ada2*wda^2*4*cos(2*wda*t) -...
%     bda2*wda^2*4*sin(2*wda*t) - ada3*wda^2*9*cos(3*wda*t) - bda3*wda^2*9*sin(3*wda*t)-ada4*wda^2*16*cos(4*wda*t) -...
%     bda4*wda^2*16*sin(4*wda*t) -ada5*wda^2*25*cos(5*wda*t) - bda5*wda^2*25*sin(5*wda*t) ;

%------------------------------------------

%---
% qd1=sin(t);
% d_qd1=cos(t);
% dd_qd1=-sin(t);
% qd2=sin(t);
% d_qd2=cos(t);
% dd_qd2=-sin(t);
% qd1=0.1*sin(t);
% d_qd1=0.1*cos(t);
% dd_qd1=-0.1*sin(t);
% qd2=0.1*sin(t);
% d_qd2=0.1*cos(t);
% dd_qd2=-0.1*sin(t);
%----------------
% q1=x(1);
% d_q1=dq(1);
% q2=x(3);
% d_q2=dq(2);
% q=[q1;q2];
% e1=qd1-q1;
% e2=qd2-q2;
% de1=d_qd1-d_q1;
% de2=d_qd2-d_q2;
% e=[e1;e2];
% de=[de1;de2];
% %---------------------
% Fai=5*eye(2);
% de=[de1;de2];
% qddot = [dd_qd1;dd_qd2]; 
% Kv=20*eye(2);%%Kv 
% Kp = 5*eye(2) ; %Kp 
% r=de+Fai*e;
% r = qddot + (Kv *de) + (Kp*e) ; %%%%
% %
% dqd=[d_qd1;d_qd2];
% dqr=dqd+Fai*e;
% ddqd=[dd_qd1;dd_qd2];
% ddqr=ddqd+Fai*de;
% f=D*ddqr+C*dqr+G+F;
% f_norm=norm(f);

sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);
end