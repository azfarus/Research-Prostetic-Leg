clear all; close all; clc; 
%link parameters

Mass = 56.7; % body weight = from Winter data
Height = 1.56; % body height = need to check
rBS =  0.1; % how many % of the residual limb that a person remained?
FOOTratio  = 0.475;
LEGratio   = 0.302;
THIGHratio = 0.323;
footLength = 0.152*Height;
ankleHeight = 0.039*Height;  % see Winter page 83 for this ratio
L1 = (sqrt(footLength^2 + 2*footLength*ankleHeight) + sqrt(footLength^2 - 2*footLength*ankleHeight))/2;
% L11 = sqrt((footLength^2 + sqrt(footLength^4 - 4*ankleHeight^2*footLength^2))/2); % this gives L1
% L12 = sqrt((footLength^2 - sqrt(footLength^4 - 4*ankleHeight^2*footLength^2))/2); % this gives d1
d1 = footLength*ankleHeight/L1;
m1 = 0.0145*Mass; r1 = L1/2; I1 = m1*(FOOTratio*L1)^2; % parameter of link 1 - FOOT
m2 = 0.0465*Mass; L2 = 0.246*Height; r2 = L2/2; I2 = m2*(LEGratio*L2)^2; % parameter of link 2 - LEG
m3 = 0.1*Mass   ; L3 = 0.245*Height; r3 = L3/2; I3 = m3*(THIGHratio*L3)^2; % parameter of link 3 - THIGH
hipHeight = ankleHeight + L2 + L3;
BFratio = 0.3; % ratio for the ball of the foot
ThFT = atan(d1/L1);
g = 9.8; % gravity acceleration