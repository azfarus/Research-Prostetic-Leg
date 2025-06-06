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
end
function [sys,x0,str,ts]=mdlInitializeSizes
global adk0 adk1 bdk1 adk2 bdk2 adk3 bdk3 adk4 bdk4 adk5 bdk5 wdk
global ada0 ada1 bda1 ada2 bda2 ada3 bda3 ada4 bda4 ada5 bda5 wda
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 6;
sizes.NumInputs = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];
%ankle coeffcient (link 2)  
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
       
 %knee coeffcient (link 1)       
       adk0 =      0.3912  ;
       adk1 =     -0.1101  ;
       bdk1 =     -0.3415  ;
       adk2 =     -0.2344 ; 
       bdk2 =      0.1558  ;
       adk3 =     0.01795 ; 
       bdk3 =     0.08599 ; 
       adk4 =   -0.007405 ; 
       bdk4 =   0.0006798;  
       adk5 =    0.003116 ; 
       bdk5 =     0.01432 ; 
       wdk =       5.601 ;
end
function sys=mdlOutputs(t,x,u)
global adk0 adk1 bdk1 adk2 bdk2 adk3 bdk3 adk4 bdk4 adk5 bdk5 wdk
global ada0 ada1 bda1 ada2 bda2 ada3 bda3 ada4 bda4 ada5 bda5 wda
qd1= adk0 + adk1*cos(wdk*t) + bdk1*sin(wdk*t) +  adk2*cos(2*wdk*t) +...
    bdk2*sin(2*wdk*t) + adk3*cos(3*wdk*t) + bdk3*sin(3*wdk*t)+adk4*cos(4*wdk*t) +...
    bdk4*sin(4*wdk*t) +adk5*cos(5*wdk*t) + bdk5*sin(5*wdk*t) ;

d_qd1= -adk1*wdk*sin(wdk*t) + bdk1*wdk*cos(wdk*t) -  adk2*2*wdk*sin(2*wdk*t) +...
    bdk2*2*wdk*cos(2*wdk*t) + adk3*3*wdk*cos(3*wdk*t) - bdk3*3*wdk*sin(3*wdk*t)+ adk4*4*wdk*cos(4*wdk*t)-...
    4*wdk*bdk4*sin(4*wdk*t) +adk5*5*wdk*cos(5*wdk*t) - bdk5*5*wdk*sin(5*wdk*t) ;


dd_qd1= -( adk1*wdk^2*cos(wdk*t) + bdk1*wdk^2*sin(wdk*t) +  adk2*wdk^2*4*cos(2*wdk*t) +...
    bdk2*wdk^2*4*sin(2*wdk*t) + adk3*wdk^2*9*cos(3*wdk*t) + bdk3*wdk^2*9*sin(3*wdk*t)+adk4*wdk^2*16*cos(4*wdk*t) +...
    bdk4*wdk^2*16*sin(4*wdk*t) +adk5*wdk^2*25*cos(5*wdk*t) + bdk5*wdk^2*25*sin(5*wdk*t)) ;



qd2= ada0 + ada1*cos(wda*t) + bda1*sin(wda*t) +  ada2*cos(2*wda*t) +...
    bda2*sin(2*wda*t) + ada3*cos(3*wda*t) + bda3*sin(3*wda*t)+ada4*cos(4*wda*t) +...
    bda4*sin(4*wda*t) +ada5*cos(5*wda*t) + bdk5*sin(5*wda*t) ;

d_qd2=-ada1*wda*sin(wda*t) + bda1*wda*cos(wda*t) -  ada2*2*wda*sin(2*wda*t) +...
    bda2*2*wda*cos(2*wda*t) + ada3*3*wda*cos(3*wda*t) - bda3*3*wda*sin(3*wda*t)+ ada4*4*wda*cos(4*wda*t)-...
    4*wda*bda4*sin(4*wda*t) +ada5*5*wda*cos(5*wda*t) - bda5*5*wda*sin(5*wda*t) ;

dd_qd2=-(ada1*wda^2*cos(wda*t) + bda1*wda^2*sin(wda*t) +  ada2*wda^2*4*cos(2*wda*t) +...
    bda2*wda^2*4*sin(2*wda*t) + ada3*wda^2*9*cos(3*wda*t) + bda3*wda^2*9*sin(3*wda*t)+ada4*wda^2*16*cos(4*wda*t) +...
    bda4*wda^2*16*sin(4*wda*t) +ada5*wda^2*25*cos(5*wda*t) + bda5*wda^2*25*sin(5*wda*t)) ;
sys(1)=qd1;
sys(2)=d_qd1;
sys(3)=dd_qd1;
sys(4)=qd2;
sys(5)=d_qd2;
sys(6)=dd_qd2;
end