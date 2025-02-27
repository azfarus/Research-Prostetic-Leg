function sys=mdlOutputs(t,x,u)
x=u(1); %Input Layer
%i=1
%j=1,2,3,4,5
%k=1
c=[-0.5 -0.25 0 0.25 0.5]; %cij
b=[0.2 0.2 0.2 0.2 0.2]’; %bj
W=ones(5,1); %Wj
h=zeros(5,1); %hj
for j=1:1:5
h(j)=exp(-norm(x-c(:,j))^2/(2*b(j)*b(j))); %HiddenLayer
end
y=W'*h; %Output Layer
sys(1)=y;
sys(2)=x;
sys(3)=h(1);
sys(4)=h(2);
sys(5)=h(3);
sys(6)=h(4);
sys(7)=h(5);
end