
% function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
% switch flag,
%   %%%%%%%%%%%%%%%%%%
%   case 0,
%     [sys,x0,str,ts]=mdlInitializeSizes;
%   case 3,
%     sys=mdlOutputs(t,x,u);
%   case {2,4,9},
%     sys=[];
%   otherwise
%     error(['Unhandled flag = ',num2str(flag)]);
% end
% end
% 
% 
% function [sys,x0,str,ts]=mdlInitializeSizes
% 
% %
% sizes = simsizes;
% 
% sizes.NumContStates  = 0;
% sizes.NumDiscStates  = 0;
% sizes.NumOutputs     = 1;
% sizes.NumInputs      = 2;
% sizes.DirFeedthrough = 1;
% sizes.NumSampleTimes = 0;   % at least one sample time is needed
% 
% sys = simsizes(sizes);
% x0  = [];
% str = [];
% ts  = [];
% end
% function sys=mdlOutputs(t,x,u)
% persistent w w_1 w_2 b ci
% alpha = 0.05 ;%momentum factor
% xite = 0.5 ; %learning rate
% 
% if t == 0 
%     b = 1.5 ;
%     ci = [-1:0.5:1;-10:5:10]; %cij
%     w=rands(5,1); %Wj
%     w_1 = w; w_2 = w_1 ; 
% end
% ut = u(1);
% y = u(2);%Input Layer
% x = [ut;y]; 
% %i=1
% %j=1,2,3,4,5
% %k=1
% % b=0.2*ones(5,1); %bj
% 
% % h=zeros(5,1); %hj
% for j=1:1:5
% h(j)=exp(-norm(x-ci(:,j))^2/(2*b^2)); %Hidden Layer
% end
% ym=w'*h'; %Output Layer
% 
% d_w = 0*w ; 
% for j = 1:1:5 
%     d_w(j) = xite*(y-ym)*h(j)
% end
% 
% w = w_1 + d_w + alpha*(w_1 - w_2) ; 
% w_2 = w_1 ; w_1 = w ; 
% 
% sys(1)=ym ;
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
sys = [];
otherwise
error(['Unhandled flag = ',num2str(flag)]);
end
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 1;
sizes.NumInputs = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[];
end
function sys=mdlOutputs(t,x,u)
persistent w w_1 w_2 b ci
alfa=0.05;
xite=0.5;
if t==0
b=1.5;
ci=[-1 -0.5 0 0.5 1;
-10 -5 0 5 10];
w=rands(5,1);
w_1=w;w_2=w_1;
end
ut=u(1);
yout=u(2);
xi=[ut yout]';
for j=1:1:5
h(j)=exp(-norm(xi-ci(:,j))^2/(2*b^2));
end
ymout=w'*h';
d_w=0*w;
for j=1:1:5 %Only weight value update
d_w(j)=xite*(yout-ymout)*h(j);
end
w=w_1+d_w+alfa*(w_1-w_2);
w_2=w_1;w_1=w;
sys(1)=ymout;
end
