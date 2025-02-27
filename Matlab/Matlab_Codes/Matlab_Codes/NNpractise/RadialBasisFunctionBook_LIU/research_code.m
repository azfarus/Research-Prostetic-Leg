
tspan = [1:0.1:10];
dydt = zeros(6,1);
y0 = dydt ; 
theta_a = dydt(1) ; theta_k = dydt(2) ; theta_h = dydt(3) ; 
[t,x] = ode45(pd1new(t,dydt), tspan, y0);

% function [xdot,M,tau1, tau2]= pd1new(t,x)
function [dydt]= pd1new(t,x)

Mass = 56.7; % body weight = from Winter data
Height = 1.56; % body height = need to check
rBS =  0.1; % how many % of the residual limb that a person remained?
FOOTratio  = 0.475;
LEGratio   = 0.302;
THIGHratio = 0.323;
footLength = 0.152*Height;
ankleHeight = 0.039*Height;  % see Winter page 83 for this ratio
Lf = (sqrt(footLength^2 + 2*footLength*ankleHeight) + sqrt(footLength^2 - 2*footLength*ankleHeight))/2;% need to check 
% L11 = sqrt((footLength^2 + sqrt(footLength^4 - 4*ankleHeight^2*footLength^2))/2); % this gives L1
% L12 = sqrt((footLength^2 - sqrt(footLength^4 - 4*ankleHeight^2*footLength^2))/2); % this gives d1
d1 = footLength*ankleHeight/Lf;
mf = 0.0145*Mass; rf = Lf/2; If = mf*(FOOTratio*Lf)^2; % parameter of link 1 - FOOT
ms = 0.0465*Mass; Ls = 0.246*Height; rs = Ls/2; Is = ms*(LEGratio*Ls)^2; % parameter of link 2 - LEG
mt = 0.1*Mass   ; Lt = 0.245*Height; rt = Lt/2; It = mt*(THIGHratio*Lt)^2; % parameter of link 3 - THIGH
hipHeight = ankleHeight + Ls + Lt;
BFratio = 0.3; % ratio for the ball of the foot
ThFT = atan(d1/Lf);
g = 9.8; % gravity acceleration

t = 1:0.1:10 ; % need to check 
T1 =2 ;T2 = 2;T3 =2 ;  amp1 = .1 ; amp2 = .1 ;amp3 = .1 ;
fact1 = 2*pi/T1 ; fact2 = 2*pi/T2 ; fact3 = 2*pi/T3 ;
sinf1 = sin(fact1*t) ; cosf2 = cos(fact2*t) ; sinf3 = sin(fact3*t)
cosf1 = cos(fact1*t) ; sinf2 = sin(fact2*t) ; cosf3 = cos(fact3*t)
qd = [amp1*sinf1 amp2*cosf2 amp3*sinf3]' ;
qdot = [fact1*amp1*cosf1 -fact2*amp2*sinf2 fact3*amp3*cosf3]';
qddot = -[(fact1)^2*amp1*sinf1 (fact2)^2*amp2*cosf2 (fact3)^2*amp3*cosf3]' ;

%tracking error 
e = qd - dydt(1:3)' ; edot = qdot - dydt(4:6)' ; 
% 
Maa = mf*rf^2 + lf ; 
Mak = mf*rf*Ls*cos(theta_a - theta_k) ; 
Mah = mf*rf*Lt*cos(theta_a - theta_h) ; 

Mka = mf*rf*Ls*cos(theta_a - theta_k) ;
Mkk = mf*Ls^2 + ms*Ls^2 + ms*rs^2+Is ; 
Mkh = (mf*Ls*Lt + ms*rs*Lt)*cos(theta_a - theta_h) ; 

Mha = mf*rf*Lt*cos(theta_a - theta_h) ;
Mhk = (mf*Ls*Lt + ms*rs*Lt)*cos(theta_k - theta_h) ;
Mhh = mf*Lt^2 + ms*Lt^2 + mt*rt^2 +It ; 



Vaa = 0 ; 
Vak = mf*rf*Ls*theta_k*sin(theta_a - theta_k) ; 
Vah = mf*rf*Lt*theta_h*sin(theta_a - theta_h) ; 

Vka = -mf*rf*Ls*theta_a*sin(theta_a - theta_k) ;
Vkk = 0 ;
Vkh = (mf*Ls*Lt + ms*rs*Lt)*theta_h*sin(theta_k - theta_h) ; 

Vha = -mf*rf*Lt*theta_a*sin(theta_a - theta_h) ;
Vhk = -(mf*Ls*Lt + ms*rs*Lt)*theta_k*sin(theta_k - theta_h) ;
Vhh = 0 ; 

Ga = mf*rf*g*sin(theta_a) ; 
Gk = (mf*Ls + ms*rs)*g*sin(theta_k) ; 
Gh = (mf*Lt + ms*Lt + mt*rt)*g*sin(theta_h) ; 



M = [Maa,Mak,Mah;Mka,Mkk,Mkh;Mha,Mhk,Mhh]; 
V = [Vaa,Vak,Vah;Vka,Vkk,Vkh;Vha,Vhk,Vhh]; 
G = [Ga ;Gk ; Gh] ; 

Tau_d = M*qdot + V*qd + G ; 


Tau = Kv*(lambda*e +edot) - Tau_d ; 
Tau1 = tau(1) ; Tau2 = tau(2); Tau3 = tau(3) ; 

dydt(1:3) = dydt(4:6); 
dydt(4:6) = -V*inv(M)*dydt(4:6) - G*inv(M) - Tau_d*inv(M) + Tau*inv(M);
dydt = [dydt(1) dydt(2) dydt(3) dydt(4) dydt(5) dydt(6)]'
end
