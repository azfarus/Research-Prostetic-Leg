% function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
% switch flag,
% case 0,
% [sys,x0,str,ts]=mdlInitializeSizes;
% case 3,
% sys=mdlOutputs(t,x,u);
% case {2,4,9}
% sys=[];
% otherwise
% error(['Unhandled flag = ',num2str(flag)]);
% end
% end
% function [sys,x0,str,ts]=mdlInitializeSizes
% sizes = simsizes;
% sizes.NumContStates = 0;
% sizes.NumDiscStates = 0;
% sizes.NumOutputs = 7;
% sizes.NumInputs = 1;
% sizes.DirFeedthrough = 1;
% sizes.NumSampleTimes = 0;
% sys = simsizes(sizes);
% x0 = [];
% str = [];
% ts = [];
% end
% function sys=mdlOutputs(t,x,u)
% x=u(1); %Input Layer
% %i=1
% %j=1,2,3,4,5
% %k=1
% c=[-0.5 -0.25 0 0.25 0.5]; %cij
% b=[0.2 0.2 0.2 0.2 0.2]'; %bj
% W=ones(5,1); %Wj
% h=zeros(5,1); %hj
% for j=1:1:5
% h(j)=exp(-norm(x-c(:,j))^2/(2*b(j)*b(j))); %Hidden Layer
% end
% y=W'*h; %Output Layer
% sys(1)=y;
% sys(2)=x;
% sys(3)=h(1);
% sys(4)=h(2);
% sys(5)=h(3);
% sys(6)=h(4);
% sys(7)=h(5);
% end


function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 3,
    sys=mdlOutputs(t,x,u);
  case {2,4,9},
    sys=[];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
end


function [sys,x0,str,ts]=mdlInitializeSizes

%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 8;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];
end
function sys=mdlOutputs(t,x,u)
x1 = u(1);
x2 = u(2);%Input Layer
x = [x1;x2]; 
%i=1
%j=1,2,3,4,5
%k=1
c=[-0.5:0.25:0.5;-0.5:0.25:0.5]; %cij
b=0.2*ones(5,1); %bj
W=ones(5,1); %Wj
h=zeros(5,1); %hj
for j=1:1:5
h(j)=exp(-norm(x-c(:,j))^2/(2*b(j)*b(j))); %Hidden Layer
end
y=W'*h; %Output Layer
sys(1)=y;
sys(2)=x1;
sys(3) =x2 ; 
sys(4)=h(1);
sys(5)=h(2);
sys(6)=h(3);
sys(7)=h(4);
sys(8)=h(5);
end