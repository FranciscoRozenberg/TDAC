clc;clear;
%% Practica 8 - Realimentacion de estados
% Matrices de estados 

Ad = [ 0.9718 0.01684 ;                 
      -2.661,  0.6921 ];

Bd = [ 0.01005 ;
       0.9477  ];

Cd = [1.0, 0];

Dd = [0];

%%
%anidamiento seguimiento de referencia
K = -acker(Ad, Bd, [.7,.7])
Acl = Ad + Bd*K;
eig(Acl)
F = (Cd *(eye(2) - (Ad + Bd *K))^-1 *Bd )^-1




