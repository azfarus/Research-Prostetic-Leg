clear all ;close all; clc; 
eta = 0.5 ; alpha = 0.05 ; 
wjo= rands(6,1) ; 
wjo_1=wjo;wjo_2=wjo_1;
wij=rands(2,6);
wij_1=wij;wij_2=wij;
dwij=0*wij;
x=[0,0]' ; 
u_1=0;
y_1=0;
I=[0,0,0,0,0,0]';
Iout=[0,0,0,0,0,0]';
FI=[0,0,0,0,0,0]';
ts=0.001;
for k=1:1:1000
time(k)=k*ts;
u(k)=0.50*sin(3*2*pi*k*ts);
y(k)=u_1^3+y_1/(1+y_1^2);
x(1)=u(k);
x(2)=y(k);
for j=1:1:6
I(j)=x'*wij(:,j);
Iout(j)=1/(1+exp(-I(j)));
end
yo(k)=wjo'*Iout; % Output of NNI networks
e(k)=y(k)-yo(k); % Error calculation
wjo=wjo_1+(eta*e(k))*Iout+alpha*(wjo_1-wjo_2);
for j=1:1:6
FI(j)=exp(-I(j))/(1+exp(-I(j)))^2;
end
for i=1:1:2
for j=1:1:6
dwij(i,j)=e(k)*eta*FI(j)*wjo(j)*x(i);
end
end
wij=wij_1+dwij+alpha*(wij_1-wij_2);
%%%%%%%%%%%%%%Jacobian%%%%%%%%%%%%%%%%
yu=0;
for j=1:1:6
yu=yu+wjo(j)*wij(1,j)*FI(j);
end
dyu(k)=yu;
wij_2=wij_1;wij_1=wij;
wjo_2=wjo_1;wjo_1=wjo;
u_1=u(k);
y_1=y(k);
end
figure(1);
plot(time,y,'r',time,yo,'b');
xlabel('times');ylabel('y and yo');
legend('y','yo')
figure(2);
plot(time,y-yo,'r');
xlabel('times');ylabel('error');
figure(3);
plot(time,dyu);
xlabel('times');ylabel('dyu');