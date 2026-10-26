%% Practica 7 - Observadores
% [TA 135] Talles de Automatizacion y Control
%% 
% 1.  Implementar un observador de estados para el angulo y velocidad angular de la barra, midiendo unicamente el angulo.
%%
% Funcion transferencia: 
%%
% $$
% \frac{\theta(t)}{u(t)} = \frac{56,28}{s^2 + 16,61 s + 158,9}
% $$
%%
num = 56.28;
den = [1 ,16.61 ,158.9];

[A,B,C,D] = tf2ss(num,den);
%%
% Matrices de espacio de estados:
disp("A:")
pretty(vpa(sym(A),4))

disp("B:")
pretty(vpa(sym(B),4))

disp("C:")
pretty(vpa(sym(C),4))

disp("D:")
pretty(vpa(sym(D),4))
%%
