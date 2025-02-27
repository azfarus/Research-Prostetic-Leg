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
 global Kv Kp
 global  g m1 m2 a1 a2
m1 = 2.6366;%5 ; 
m2 = 0.82;%2.6366+0.82 ;
a1 = 0.19;%0.38 ;
a2 = 0.0608;%0.0608+0.38 ;
g=9.8;
% node=7;
% c=[-1.5 -1 -0.5 0 0.5 1 1.5;
% -1.5 -1 -0.5 0 0.5 1 1.5];
% b=10;
% Fai=5*eye(2);
Kv=20*eye(2);%%Kv 
Kp = 100*eye(2) ; %Kp 
sizes = simsizes;
sizes.NumContStates = 2;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 2;%
sizes.NumInputs = 10;%
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = zeros(1,2);%
str = [];
ts = [0 0];
end
function sys=mdlDerivatives(t,x,u)
% global node c b Fai
global Kv Kp 
global  g m1 m2 a1 a2
qd1=u(1);
d_qd1=u(2);
dd_qd1=u(3);
qd2=u(4);
d_qd2=u(5);
dd_qd2=u(6);
q1=u(7);
d_q1=u(8);
q2=u(9);
d_q2=u(10);
q=[q1;q2];
e1=qd1-q1;
e2=qd2-q2;
de1=d_qd1-d_q1;
de2=d_qd2-d_q2;
e=[e1;e2];
de=[de1;de2];
%
% r=de+Fai*e;
qd=[qd1;qd2];
dqd=[d_qd1;d_qd2];
qddot = [dd_qd1;dd_qd2] ;
r = qddot + (Kv *de) + (Kp*e) ;

x = [u(7) u(8) u(9) u(10)] ; 

M = [(m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(3)), m2*a2^2 + m2*a1*a2*cos(x(3)); m2*a2^2 + m2*a1*a2*cos(x(3)), m2*a2^2];
V = [ -m2*a1*a2*(2*x(2)*x(4)+(x(4))^2)*sin(x(3));m2*a1*a2*(x(2))^2*sin(x(3))] ; 
G = [(m1+m2)*g*a1*cos(x(1))+m2*g*a2*cos(x(1)+x(3)); m2*g*a2*cos(x(1)+x(3))]; 

% M11 = u(11) ; M12 = u(12) ;M21 = u(13) ; M22 =u(14) ; %M matrix 
% M = [M11,M12;M21,M22] ; 
% 
% N1 =u(15);N2 = u(16); % Nonlinear terms 
% N = [N1;N2] ; 
N = V + G ; 
tol = M*r + N ; 
sys(1)=tol(1);
sys(2)=tol(2);
end
function sys=mdlOutputs(t,x,u)
% global node c b Fai
global Kv Kp
global  g m1 m2 a1 a2
qd1=u(1);
d_qd1=u(2);
dd_qd1=u(3);
qd2=u(4);
d_qd2=u(5);
dd_qd2=u(6);
q1=u(7);%theta1
d_q1=u(8);%theta1dot
q2=u(9);%theta2
d_q2=u(10);%theta2dot
q=[q1;q2];
qddot = [dd_qd1;dd_qd2] ;
e1=qd1-q1;
e2=qd2-q2;
de1=d_qd1-d_q1;
de2=d_qd2-d_q2;
e=[e1;e2];
de=[de1;de2];
% Kv=20*eye(2);%%Kv 
% Kp = 5*eye(2) ; %Kp 
% r=de+Fai*e;
r = qddot + (Kv*de) + (Kp*e) ; %%%%
% % % 
% M11 = u(11) ; M12 = u(12) ;M21 = u(13) ; M22 =u(14) ; %M matrix 
% M = [M11,M12;M21,M22] ; 
% 
% N1 =u(15);N2 = u(16); % Nonlinear terms 
% N = [N1;N2] ; 

% 
% qd=[qd1;qd2];
% dqd=[d_qd1;d_qd2];
% dqr=dqd+Fai*e;
% ddqd=[dd_qd1;dd_qd2];
% ddqr=ddqd+Fai*de;
% W_f1=[x(1:node)]';
% W_f2=[x(node+1:node*2)]';
% z1=[e(1);de(1)];
% z2=[e(2);de(2)];
% for j=1:1:node
% h1(j)=exp(-norm(z1-c(:,j))^2/(b*b));
% h2(j)=exp(-norm(z2-c(:,j))^2/(b*b));
% end
% fn=[W_f1*h1';
% W_f2*h2'];
% Kv=20*eye(2);
% epN=0.20;bd=0.1;
% v=-(epN+bd)*sign(r);
% tol=fn+Kv*r-v;
% fn_norm=norm(fn);
x = [u(7) u(8) u(9) u(10)] ; 

M = [(m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(3)), m2*a2^2 + m2*a1*a2*cos(x(3)); m2*a2^2 + m2*a1*a2*cos(x(3)), m2*a2^2];
V = [ -m2*a1*a2*(2*x(2)*x(4)+(x(4))^2)*sin(x(3));m2*a1*a2*(x(2))^2*sin(x(3))] ; 
G = [(m1+m2)*g*a1*cos(x(1))+m2*g*a2*cos(x(1)+x(3)); m2*g*a2*cos(x(1)+x(3))]; 
N = V+G ; 
tol = M*r + N ; 
sys(1)=tol(1);
sys(2)=tol(2);
end