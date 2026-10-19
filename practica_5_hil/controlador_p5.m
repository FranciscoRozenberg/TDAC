clear all;close all;clc 
%BODE
optionss=bodeoptions; 
optionss.MagVisible='on'; 
optionss.PhaseMatching='on'; 
optionss.PhaseMatchingValue=-180; 
optionss.PhaseMatchingFreq=1; 
optionss.Grid='on'; 
optionss.XLim = [0.00001 1000000];

%DINAMICA DE LA PLANTA
g=9.8;
Qi=8;
d=0.0165;
L1=0.1;
L2=0.4;
L=0.9;

x=sym('x',[1 1],'real'); 
u=sym('u','real'); 

f = [(-sqrt(abs(2*g*x))*d^2/4*u*pi + Qi)/(L1+(L2-L1)*x/L)^2];
A=jacobian(f,x); 
B=jacobian(f,u); 
C=[1]; 
D=0;

x0=0.45;
u0_hat=(L1+(L2-L1)*x0/L)^2/((-sqrt(abs(2*g*x0))*d^2/4*pi + Qi));   
A=subs(A,[x;u],[x0;u0_hat]); 
B=subs(B,[x;u],[x0;u0_hat]); 
A=double(A); 
B=double(B); 
% P=zpk(ss(A,B,C,D));
P=zpk([],[-0.002397],-0.004233);

C = pidtune(P,'PID');
C_z = c2d(C,1,'tustin');

Kp=-4.911;
Ki=-0.0102;
Ts=1;
num = [Kp + Ki*Ts/2, -Kp + Ki*Ts/2];
den = [1, -1];
Cz = tf(num, den, Ts, 'Variable', 'z');

