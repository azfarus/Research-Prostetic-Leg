clear all; close all; clc; 
eta = 0.1 ; alp = 0.5 ; 
wj0 = rand(6,1) ; wj01 = wj0 ; wj02= wj01 ; 
wij = rand(2,6) ; wij1 = wij ; wij2= wij1 ;
dwij=0*wij; 
x = [0;0] ; u1 = 0 ; y1 = 0 ; 
I = zeros(6,1) ; Iout = zeros(6,1) ; FI = zeros(6,1) ; 
ts=0.001;
for k=1:1:1000
time(k)=k*ts;

    u(k)=0.50*sin(3*2*pi*k*ts);
    y(k)=u1^3+y1/(1+y1^2);
    x = [u(k);y(k)] ;
    for j = 1:6 
        I(j) = x'*wij(:,j);
        Iout(j)=1/(1+exp(-I(j)));
    end
    yo(k) = wj0'*Iout ; 
    e(k)=y(k)-yo(k);
    wj0=wj01+(eta*e(k))*Iout+alp*(wj01-wj02);
    for j=1:1:6
      FI(j)=exp(-I(j))/(1+exp(-I(j)))^2;
    end
    for i=1:1:2     
      for j=1:1:6
       dwij(i,j)=e(k)*eta*FI(j)*wj0(j)*x(i);
      end
    end
    wij=wij1+dwij+alp*(wij1-wij2);
    yu=0;
    for j=1:1:6
      yu=yu+wj0(j)*wij(1,j)*FI(j);
    end

dyu(k)=yu;
wij2=wij1;wij1=wij;
wj02=wj01;wj01=wj0;
u1=u(k);
y1=y(k);
end
figure(1);
plot(time,y,'r',time,yo,'b');
xlabel('times');ylabel('y and yo');
figure(2);
plot(time,y-yo,'r');
xlabel('times');ylabel('error');
figure(3);
plot(time,dyu);
xlabel('times');ylabel('dyu');
    
