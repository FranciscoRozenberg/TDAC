clear; close all; clc
out = load("con_proporcional.mat");

t = out.out.tout(800:2000);

% Modelo continuo identificado de la planta de 4 polos
p1 = 0.6202;
p2 = -6.4049;
p3 = -8.3056 + 9.4818i;
p4 = -8.3056 - 9.4818i;

% Ganancia de la planta
K = 16.03*56.28;

% Construir la función de transferencia
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Ver la planta
G
pole(G)

Kp = 3.7
Kd = 0.001
%Ki = 0;

%L = c2d(feedback((Kp + Ki/s) * G,1), 20e-3, 'Tustin')
L = feedback((Kp + Kd*s) * G,1)

%u_sym(767) = 150;

% Simular el modelo con la entrada REAL medida

t_rel = t - t(1);
%y_sim = real(lsim(L, u_sym, t_rel));

% Comparar variaciones respecto al valor inicial
%y_sim_var = y_sim - y_sim(1);

% Graficar las dos respuestas
figure;
impulse(L)
%plot(y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Tiempo [s]');
ylabel('Variación de la salida');
title('Simulacion con control PI');
legend('Medición real', ...
    'Location', 'best');