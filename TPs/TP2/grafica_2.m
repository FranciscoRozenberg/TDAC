clear; close all; clc

out = load("Escalon_servo_barra.mat");

u = out.out.d5(20:280)/8;
t = out.out.tout(20:280);

% Asegurar que el tiempo comience en cero
t = t - t(1);

%% Planta 1: modelo original
p1 = 0;
p2 = -0.8468;
p3 = -8.3056 + 9.4818i;
p4 = -8.3056 - 9.4818i;

K1 = 4.446 * 56.28;

s = tf('s');
G1 = K1 / ((s-p1)*(s-p2)*(s-p3)*(s-p4));

%% Planta 2: modelo con los polos identificados considerando el retardo
p1 = 0;
p2 = -0.8468;
p3 = -9.9692 + 9.8746i;
p4 = -9.9692 - 9.8746i;

K2 = 4.446 * 70.01;

G2 = K2 / ((s-p1)*(s-p2)*(s-p3)*(s-p4));


N = length(u);
u_sym = zeros(1,N);
u_sym(50) = 5;
%% Simulación de ambas plantas ante la misma entrada
y1 = lsim(G1, u_sym, t);
y2 = lsim(G2, u_sym, t);

%% Gráficos
figure;

subplot(2,1,1)
plot(t, u_sym, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('u(t)');
title('Escalon de entrada');

subplot(2,1,2)
plot(t, y1, 'b', 'LineWidth', 1.4);
hold on;
plot(t, y2, 'r--', 'LineWidth', 1.4);
grid on;
xlabel('Tiempo [s]');
ylabel('Salida');
title('Respuesta al escalon de ambas plantas');
legend('Planta 1: modelo original', ...
       'Planta 2: modelo reajustado', ...
       'Location', 'best');