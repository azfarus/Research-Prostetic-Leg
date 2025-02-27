
clear all; close all ;clc; 
ada0 =   -0.007782 ; %(-0.008938, -0.006626)
       ada1 =     -0.0204  ;%(-0.02222, -0.01857)
       bda1 =      0.0984  ;%(0.09676, 0.1)
       ada2 =     0.01139  ;%(0.008884, 0.01389)
       bda2 =     -0.1117  ;%(-0.1134, -0.1101)
       ada3 =    -0.06086  ;%(-0.06254, -0.05917)
       bda3 =     0.01065  ;%(0.008338, 0.01296)
       ada4 =     0.03639  ;%(0.03473, 0.03805)
       bda4 =    0.002878  ;%(0.0008274, 0.004929)
       ada5 =    0.006763 ; %(0.004849, 0.008678)
       bda5 =    -0.02346 ; %(-0.02511, -0.02181)
       wda =         5.6  ;%(5.595, 5.605)
  
       t = 0:0.01:10;
       qd2= ada0 + ada1*cos(wda*t) + bda1*sin(wda*t) +  ada2*cos(2*wda*t) +...
    bda2*sin(2*wda*t) + ada3*cos(3*wda*t) + bda3*sin(3*wda*t)+ada4*cos(4*wda*t) +...
    bda4*sin(4*wda*t) +ada5*cos(5*wda*t) + bda5*sin(5*wda*t) ;

d_qd2=-ada1*wda*sin(wda*t) + bda1*wda*cos(wda*t) -  ada2*2*wda*sin(2*wda*t) +...
    bda2*2*wda*cos(2*wda*t) - ada3*3*wda*sin(3*wda*t) - bda3*3*wda*cos(3*wda*t)- ada4*4*wda*sin(4*wda*t)+...
    4*wda*bda4*cos(4*wda*t) -ada5*5*wda*sin(5*wda*t) + bda5*5*wda*cos(5*wda*t) ;

dd_qd2= -ada1*wda^2*cos(wda*t) - bda1*wda^2*sin(wda*t) -  ada2*wda^2*4*cos(2*wda*t) -...
    bda2*wda^2*4*sin(2*wda*t) - ada3*wda^2*9*cos(3*wda*t) - bda3*wda^2*9*sin(3*wda*t)-ada4*wda^2*16*cos(4*wda*t) -...
    bda4*wda^2*16*sin(4*wda*t) -ada5*wda^2*25*cos(5*wda*t) - bda5*wda^2*25*sin(5*wda*t) ;
figure(1) 
plot(t,qd2)
figure(2) 
plot(t,d_qd2)
figure(3) 
plot(t,dd_qd2)

