%desired trajectory parameter
% % General model Fourier5:ankle which is link 2 
   clear all; close all; clc; 
% Coefficients (with 95% confidence bounds):
       a0 =   -0.007782 ; %(-0.008938, -0.006626, )
       a1 =     -0.0204 ;%(-0.02222, -0.01857)
       b1 =      0.0984 ;%(0.09676, 0.1) 
       a2 =    0.01139;%(0.008884, 0.01389) 
       b2 =     -0.1117  ;%(-0.1134, -0.1101)
       a3 =    -0.06086  ;%(-0.06254, -0.05917)
       b3 =     0.01065  ;%(0.008338, 0.01296)
       a4 =     0.03639  ;%(0.03473, 0.03805)
       b4 =    0.002878  ;%(0.0008274, 0.004929)
       a5 =    0.006763 ; %(0.004849, 0.008678)
       b5 =    -0.02346 ; %(-0.02511, -0.02181)
       w =         5.6  ;%(5.595, 5.605)
       %
   x=(0:0.001:3.5)     
 f_x =  a0 + a1*cos(x*w) + b1*sin(x*w) + a2*cos(2*x*w) + b2*sin(2*x*w) + a3*cos(3*x*w) + b3*sin(3*x*w) + a4*cos(4*x*w) + b4*sin(4*x*w) + a5*cos(5*x*w) + b5*sin(5*x*w);

 f_x2 = f_x -(f_x(1))
           plot(x,f_x2)
%%
% clear all; close all; clc; 
%  x=(0:0.001:3.5) %knee which is link 1 
%  a0 =      0.3912  
%        a1 =     -0.1101  
%        b1 =     -0.3415  
%        a2 =     -0.2344  
%        b2 =      0.1558  
%        a3 =     0.01795  
%        b3 =     0.08599  
%        a4 =   -0.007405  
%        b4 =   0.0006798  
%        a5 =    0.003116  
%        b5 =     0.01432  
%        w =       5.601  
%      f_x =  a0 + a1*cos(x*w) + b1*sin(x*w) + a2*cos(2*x*w) + b2*sin(2*x*w) + a3*cos(3*x*w) + b3*sin(3*x*w) + a4*cos(4*x*w) + b4*sin(4*x*w) + a5*cos(5*x*w) + b5*sin(5*x*w)
%              
% % Coefficients (with 95% confidence bounds):
%        
%          plot(x,f_x)