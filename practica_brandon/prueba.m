clc;clear;close all;

G_bc = tf(4.446, [1 0.8468 0]);
G_sb = tf(56.28, [1 16.61 158.9]);

G = series(G_bc, G_sb);
disp('TRANSFERENCIA DE LA PLANTA COMPLETA');
G
[num, den] = tfdata(G, 'v');
disp('Numerador original:');
disp(num);
disp('Denominador original:');
disp(den);

sys = ss(G);
[A, B, C, D] = ssdata(sys);
disp('MATRICES EN VARIABLES DE ESTADO');
disp('Matriz A:');
disp(A);
disp('Matriz B:');
disp(B);
disp('Matriz C:');
disp(C);
disp('Matriz D:');
disp(D);

% G(s) = C*(sI-A)^(-1)*B + D
G_reconstruida = tf(ss(A, B, C, D));

disp('TRANSFERENCIA RECONSTRUIDA');
G_reconstruida

disp('Polos de la transferencia original:');
disp(pole(G));
disp('Polos del modelo reconstruido:');
disp(pole(G_reconstruida));
